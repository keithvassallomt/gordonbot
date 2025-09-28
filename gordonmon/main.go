package main

import (
	"bufio"
	"context"
	"encoding/json"
	"fmt"
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"regexp"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/charmbracelet/bubbles/spinner"
	"github.com/charmbracelet/bubbles/viewport"
	tea "github.com/charmbracelet/bubbletea"
	"github.com/charmbracelet/lipgloss"
)

// ---------- Config ----------

type cfg struct {
	root             string
	backendDir       string
	frontendDir      string
	mediamtxDir      string
	mediamtxCmd      string
	backendPort      int
	frontendPort     int
	whepHTTPPort     int
	rtspPort         int
	apiHost          string
	backendAccessLog bool
}

func getenv(key, def string) string {
	v := os.Getenv(key)
	if v == "" {
		return def
	}
	return v
}

func getenvBool(key string, def bool) bool {
	if v := os.Getenv(key); v != "" {
		switch strings.ToLower(strings.TrimSpace(v)) {
		case "1", "true", "yes", "on":
			return true
		case "0", "false", "no", "off":
			return false
		}
	}
	return def
}

func getenvInt(key string, def int) int {
	if v := os.Getenv(key); v != "" {
		if n, err := strconv.Atoi(v); err == nil {
			return n
		}
	}
	return def
}

func detectRoot() string {
	// repo root = parent of this gordonmon dir by default
	wd, _ := os.Getwd()
	if strings.HasSuffix(wd, string(os.PathSeparator)+"gordonmon") {
		return filepath.Dir(wd)
	}
	// fallback: current dir
	return wd
}

func loadCfg() cfg {
	root := detectRoot()
	return cfg{
		root:             root,
		backendDir:       getenv("BACKEND_DIR", filepath.Join(root, "gordonbot-backend")),
		frontendDir:      getenv("FRONTEND_DIR", filepath.Join(root, "gordonbot-dashboard")),
		mediamtxDir:      getenv("MEDIAMTX_DIR", "/home/keith/mediamtx"),
		mediamtxCmd:      getenv("MEDIAMTX_CMD", filepath.Join(getenv("MEDIAMTX_DIR", "/home/keith/mediamtx"), "mediamtx")),
		backendPort:      getenvInt("BACKEND_PORT", 8000),
		frontendPort:     getenvInt("FRONTEND_PORT", 5173),
		whepHTTPPort:     getenvInt("MEDIAMTX_HTTP_PORT", 8889),
		rtspPort:         getenvInt("MEDIAMTX_RTSP_PORT", 8554),
		apiHost:          getenv("GORDONMON_API_HOST", "127.0.0.1"),
		backendAccessLog: getenvBool("GORDONMON_BACKEND_ACCESS_LOG", false),
	}
}

func backendUvicornCommand(c cfg) string {
	cmd := fmt.Sprintf("uvicorn app.main:app --host 0.0.0.0 --port %d --reload", c.backendPort)
	if !c.backendAccessLog {
		cmd += " --no-access-log --log-level warning"
	}
	return cmd
}

// ---------- Process management ----------

type proc struct {
	name   string
	cmd    *exec.Cmd
	cancel context.CancelFunc
	mu     sync.Mutex
	alive  bool
}

func (p *proc) isAlive() bool {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.alive
}

func (p *proc) setAlive(v bool) { p.mu.Lock(); p.alive = v; p.mu.Unlock() }

// global program handle for sending messages from goroutines
var program *tea.Program
var procsMu sync.Mutex
var procs = map[string]*proc{}
var logFileMu sync.Mutex
var logFile *os.File
var logFilePath string

// telemetry message from pollers
type telemetryMsg struct {
	cpu   *float64
	pct   *float64
	state string
}
type telemetryTickMsg struct{}

// ---------- Messages ----------

type logMsg struct{ line string }
type statusMsg struct{ text string }
type startedMsg struct {
	what string
	ok   bool
	err  error
}
type procExitMsg struct{ name string }
type ipUpdateMsg struct{ ip string }

// ---------- Utilities ----------

func firstNonLoopbackIPv4() string {
	// Try net.Interfaces
	ifaces, _ := net.Interfaces()
	for _, ifc := range ifaces {
		if (ifc.Flags & net.FlagUp) == 0 {
			continue
		}
		addrs, _ := ifc.Addrs()
		for _, a := range addrs {
			if ipnet, ok := a.(*net.IPNet); ok {
				ip := ipnet.IP
				if ip == nil || ip.IsLoopback() {
					continue
				}
				ip4 := ip.To4()
				if ip4 != nil {
					return ip4.String()
				}
			}
		}
	}
	// Fallback: hostname -I
	out, _ := exec.Command("bash", "-lc", "hostname -I").Output()
	fields := strings.Fields(string(out))
	for _, f := range fields {
		if !strings.HasPrefix(f, "127.") {
			return f
		}
	}
	return "127.0.0.1"
}

func initLogFile(c cfg) {
	logsDir := filepath.Join(c.root, "gordonmon", "logs")
	if err := os.MkdirAll(logsDir, 0o755); err != nil {
		logsDir = filepath.Join("logs")
		if err := os.MkdirAll(logsDir, 0o755); err != nil {
			return
		}
	}
	name := fmt.Sprintf("gordonmon-%s.log", time.Now().Format("2006-01-02_15-04-05"))
	path := filepath.Join(logsDir, name)
	f, err := os.OpenFile(path, os.O_CREATE|os.O_WRONLY|os.O_APPEND, 0o644)
	if err != nil {
		return
	}
	logFileMu.Lock()
	logFile = f
	logFilePath = path
	logFileMu.Unlock()
	writeLogLine(fmt.Sprintf("[start] logging to %s", path))
}

func writeLogLine(line string) {
	logFileMu.Lock()
	defer logFileMu.Unlock()
	if logFile == nil {
		return
	}
	_, _ = logFile.WriteString(line + "\n")
}

func closeLogFile() {
	logFileMu.Lock()
	defer logFileMu.Unlock()
	if logFile != nil {
		_ = logFile.Close()
		logFile = nil
	}
	logFilePath = ""
}

func haveBin(bin string) bool { return exec.Command("bash", "-lc", "command -v "+bin).Run() == nil }

func killPort(port int) {
	// Try lsof, else fuser
	if haveBin("lsof") {
		cmd := fmt.Sprintf("pids=$(lsof -iTCP:%d -sTCP:LISTEN -t 2>/dev/null || true); if [ -n \"$pids\" ]; then kill $pids 2>/dev/null || true; sleep 0.5; for p in $pids; do kill -9 $p 2>/dev/null || true; done; fi", port)
		exec.Command("bash", "-lc", cmd).Run()
		return
	}
	if haveBin("fuser") {
		exec.Command("bash", "-lc", fmt.Sprintf("fuser -k %d/tcp || true", port)).Run()
	}
}

func mediamtxRunning(httpPort int, cmdPath string) bool {
	if haveBin("pgrep") {
		if exec.Command("bash", "-lc", "pgrep -x mediamtx").Run() == nil {
			return true
		}
		if cmdPath != "" {
			cmd := fmt.Sprintf("pgrep -f %q", cmdPath)
			if exec.Command("bash", "-lc", cmd).Run() == nil {
				return true
			}
		}
	}
	if haveBin("lsof") {
		if exec.Command("bash", "-lc", fmt.Sprintf("lsof -iTCP:%d -sTCP:LISTEN -t >/dev/null", httpPort)).Run() == nil {
			return true
		}
	}
	return false
}

func startProcess(name string, script string, workdir string) (*proc, error) {
	ctx, cancel := context.WithCancel(context.Background())
	cmd := exec.CommandContext(ctx, "bash", "-lc", script)
	cmd.Dir = workdir
	stdout, _ := cmd.StdoutPipe()
	stderr, _ := cmd.StderrPipe()
	if err := cmd.Start(); err != nil {
		cancel()
		return nil, err
	}
	p := &proc{name: name, cmd: cmd, cancel: cancel}
	p.setAlive(true)
	procsMu.Lock()
	procs[name] = p
	procsMu.Unlock()

	// Stream stdout
	go func() {
		scanner := bufio.NewScanner(stdout)
		for scanner.Scan() {
			if program != nil {
				program.Send(logMsg{fmt.Sprintf("[%s] %s", name, scanner.Text())})
			}
		}
	}()
	// Stream stderr
	go func() {
		scanner := bufio.NewScanner(stderr)
		for scanner.Scan() {
			if program != nil {
				program.Send(logMsg{fmt.Sprintf("[%s] %s", name, scanner.Text())})
			}
		}
	}()
	// Waiter
	go func() {
		_ = cmd.Wait()
		p.setAlive(false)
		if program != nil {
			program.Send(procExitMsg{name: name})
		}
	}()
	return p, nil
}

func stopProcess(p *proc) {
	if p == nil {
		return
	}
	p.mu.Lock()
	cmd := p.cmd
	cancel := p.cancel
	p.mu.Unlock()
	if cmd == nil {
		return
	}
	// Try SIGTERM then SIGKILL
	_ = cmd.Process.Signal(os.Interrupt)
	time.Sleep(400 * time.Millisecond)
	_ = cmd.Process.Kill()
	if cancel != nil {
		cancel()
	}
}

func stopAll() {
	procsMu.Lock()
	m := make(map[string]*proc, len(procs))
	for k, v := range procs {
		m[k] = v
	}
	procsMu.Unlock()
	for _, p := range m {
		stopProcess(p)
	}
}

// ---------- UI Model ----------

type model struct {
	cfg            cfg
	vp             viewport.Model
	spinner        spinner.Model
	logs           []string
	maxLines       int
	status         string
	ip             string
	backend        *proc
	frontend       *proc
	mediamtx       *proc
	starting       bool
	width          int
	height         int
	ready          bool
	headerH        int
	footerH        int
	tailFollow     bool
	mouseEnabled   bool
	levelThreshold int
	showLevelModal bool
	modalIndex     int
	// telemetry
	cpuTemp   *float64
	battPct   *float64
	battState string
}

var (
	titleStyle  = lipgloss.NewStyle().Bold(true).Foreground(lipgloss.Color("212"))
	okStyle     = lipgloss.NewStyle().Foreground(lipgloss.Color("42"))
	warnStyle   = lipgloss.NewStyle().Foreground(lipgloss.Color("178"))
	errStyle    = lipgloss.NewStyle().Foreground(lipgloss.Color("196"))
	faintStyle  = lipgloss.NewStyle().Foreground(lipgloss.Color("245"))
	tagStart    = lipgloss.NewStyle().Foreground(lipgloss.Color("117")).Render("[start]")
	tagBackend  = lipgloss.NewStyle().Foreground(lipgloss.Color("111")).Render("[backend]")
	tagFrontend = lipgloss.NewStyle().Foreground(lipgloss.Color("69")).Render("[frontend]")
	tagMTX      = lipgloss.NewStyle().Foreground(lipgloss.Color("177")).Render("[mediamtx]")

	// Panels
	borderColor = lipgloss.Color("238")
	headerBox   = lipgloss.NewStyle().Border(lipgloss.RoundedBorder()).BorderForeground(borderColor).Padding(0, 1)
	bodyBox     = lipgloss.NewStyle().Border(lipgloss.NormalBorder()).BorderForeground(borderColor).Padding(0, 1)
	footerBox   = lipgloss.NewStyle().Border(lipgloss.RoundedBorder()).BorderForeground(borderColor).Padding(0, 1)
)

func initialModel(c cfg, pagerMode bool) model {
	sp := spinner.New()
	sp.Style = faintStyle
	vp := viewport.New(80, 20)
	vp.MouseWheelEnabled = true
	// pagerMode=true means: do NOT auto-follow logs (act like a pager)
	return model{cfg: c, vp: vp, spinner: sp, logs: make([]string, 0, 200), maxLines: 5000, status: "starting", ip: firstNonLoopbackIPv4(), starting: true, tailFollow: !pagerMode, mouseEnabled: true, levelThreshold: 20, showLevelModal: false, modalIndex: 1}
}

func (m model) Init() tea.Cmd {
	return tea.Batch(m.spinner.Tick, startAllCmd(m.cfg), scheduleTelemetry(), logFileNoticeCmd())
}

// ----- Commands -----

func logFileNoticeCmd() tea.Cmd {
	logFileMu.Lock()
	path := logFilePath
	logFileMu.Unlock()
	if path == "" {
		return nil
	}
	return func() tea.Msg {
		return logMsg{fmt.Sprintf("[start] Saving log copy to %s", path)}
	}
}

func startAllCmd(c cfg) tea.Cmd {
	return func() tea.Msg {
		// clear ports
		program.Send(logMsg{fmt.Sprintf("%s Killing existing processes on backend port %d", tagStart, c.backendPort)})
		killPort(c.backendPort)
		program.Send(logMsg{fmt.Sprintf("%s Killing existing processes on frontend port %d", tagStart, c.frontendPort)})
		killPort(c.frontendPort)

		// MediaMTX
		if !mediamtxRunning(c.whepHTTPPort, c.mediamtxCmd) && c.mediamtxCmd != "" {
			program.Send(logMsg{fmt.Sprintf("%s Starting MediaMTX…", tagStart)})
			p, err := startProcess("mediamtx", fmt.Sprintf("stdbuf -oL -eL %q", c.mediamtxCmd), c.mediamtxDir)
			if err != nil {
				program.Send(logMsg{fmt.Sprintf("%s Failed to start MediaMTX: %v", tagStart, err)})
			} else {
				_ = p
			}
		} else {
			program.Send(logMsg{fmt.Sprintf("%s MediaMTX already running; will not start a new instance", tagStart)})
		}

		// Backend
		backendArgs := backendUvicornCommand(c)
		beCmd := ""
		if _, err := os.Stat(filepath.Join(c.backendDir, ".venv", "bin", "activate")); err == nil {
			beCmd = "source .venv/bin/activate && stdbuf -oL -eL " + backendArgs
		} else {
			beCmd = "stdbuf -oL -eL " + backendArgs
		}
		bp, beErr := startProcess("backend", beCmd, c.backendDir)
		if beErr != nil {
			program.Send(logMsg{fmt.Sprintf("%s Backend failed to start: %v", tagStart, beErr)})
			return startedMsg{what: "all", ok: false, err: beErr}
		}
		_ = bp

		// Frontend
		feCmd := "stdbuf -oL -eL npm run dev -- --host --port " + strconv.Itoa(c.frontendPort)
		fp, feErr := startProcess("frontend", feCmd, c.frontendDir)
		if feErr != nil {
			program.Send(logMsg{fmt.Sprintf("%s Frontend failed to start: %v", tagStart, feErr)})
			return startedMsg{what: "all", ok: false, err: feErr}
		}
		_ = fp

		return startedMsg{what: "all", ok: true}
	}
}

func reloadCmd(c cfg) tea.Cmd { return startAllCmd(c) }

// ----- Update -----

func (m model) Update(msg tea.Msg) (tea.Model, tea.Cmd) {
	var cmds []tea.Cmd
	switch msg := msg.(type) {
	case tea.WindowSizeMsg:
		m.width, m.height = msg.Width, msg.Height
		if m.width <= 0 {
			m.width = 80
		}
		if m.height <= 0 {
			m.height = 24
		}
		// Compute exact inner sizes based on box frames
		hbFw, _ := headerBox.GetFrameSize()
		fbFw, _ := footerBox.GetFrameSize()
		bbFw, bbFh := bodyBox.GetFrameSize()
		innerHeaderW := max(1, m.width-hbFw)
		innerFooterW := max(1, m.width-fbFw)
		headerRendered := headerBox.Width(m.width).Render(clipWidth(headerLines(m), innerHeaderW))
		footerRendered := footerBox.Width(m.width).Render(footerContent(innerFooterW, levelNameFromThreshold(m.levelThreshold), m.mouseEnabled))
		m.headerH = lipgloss.Height(headerRendered)
		m.footerH = lipgloss.Height(footerRendered)
		vpH := m.height - m.headerH - m.footerH - bbFh
		if vpH < 3 {
			vpH = 3
		}
		m.vp.Style = lipgloss.NewStyle() // no border here; we'll wrap when rendering
		m.vp.Width = max(10, m.width-bbFw)
		m.vp.Height = vpH
		m.vp.YPosition = m.headerH // anchor mouse scrolling below header
		m.updateContent()
		m.ready = true
	case tea.KeyMsg:
		if m.showLevelModal {
			key := strings.ToLower(msg.String())
			switch key {
			case "esc":
				m.showLevelModal = false
				return m, nil
			case "up", "k":
				if m.modalIndex > 0 {
					m.modalIndex--
				}
				return m, nil
			case "down", "j":
				if m.modalIndex < len(levelNames())-1 {
					m.modalIndex++
				}
				return m, nil
			case "enter":
				cmd := m.applySelectedLevel()
				return m, cmd
			case "1", "2", "3", "4", "5":
				idx := int(key[0] - '1')
				if idx >= 0 && idx < len(levelNames()) {
					m.modalIndex = idx
					cmd := m.applySelectedLevel()
					return m, cmd
				}
				return m, nil
			}
		}
		switch strings.ToLower(msg.String()) {
		case "q", "ctrl+c":
			return m, tea.Quit
		case "r":
			m.status = "restarting"
			m.starting = true
			cmds = append(cmds, reloadCmd(m.cfg))
		case "b":
			m.status = "restarting"
			m.starting = true
			cmds = append(cmds, restartBackendCmd(m.cfg))
		case "f":
			m.status = "restarting"
			m.starting = true
			cmds = append(cmds, restartFrontendCmd(m.cfg))
		case "p":
			// toggle pager-like mode: when off, we auto-follow; when on, we keep position
			m.tailFollow = !m.tailFollow
		case "s":
			m.mouseEnabled = !m.mouseEnabled
			if m.mouseEnabled {
				m.vp.MouseWheelEnabled = true
				cmds = append(cmds, tea.EnableMouseCellMotion)
				m.appendLog("[start] Mouse capture enabled; scroll wheel active. Press S to enable selection.")
			} else {
				m.vp.MouseWheelEnabled = false
				cmds = append(cmds, tea.DisableMouse)
				m.appendLog("[start] Mouse capture disabled; text selectable. Press S to re-enable scrolling.")
			}
			m.updateContent()
		case "l":
			m.showLevelModal = true
			m.modalIndex = levelIndexFromThreshold(m.levelThreshold)
		default:
			// pass to viewport for navigation
			var c tea.Cmd
			m.vp, c = m.vp.Update(msg)
			if c != nil {
				cmds = append(cmds, c)
			}
		}
	case spinner.TickMsg:
		var c tea.Cmd
		m.spinner, c = m.spinner.Update(msg)
		cmds = append(cmds, c)
	case logMsg:
		m.appendLog(msg.line)
		m.updateContent()
	case startedMsg:
		if msg.ok {
			m.status = "running"
		} else {
			m.status = "error"
		}
		m.starting = false
	case procExitMsg:
		m.appendStyledLog(fmt.Sprintf("[%s] exited", msg.name), errStyle)
		m.updateContent()
	case ipUpdateMsg:
		m.ip = msg.ip
	case telemetryMsg:
		m.cpuTemp = msg.cpu
		m.battPct = msg.pct
		if msg.state != "" {
			m.battState = msg.state
		}
	case telemetryTickMsg:
		// schedule next poll and trigger fetch
		cmds = append(cmds, scheduleTelemetry(), fetchTelemetryCmd(m.cfg))
	default:
		// pass to viewport
		var c tea.Cmd
		m.vp, c = m.vp.Update(msg)
		if c != nil {
			cmds = append(cmds, c)
		}
	}
	return m, tea.Batch(cmds...)
}

// ----- Restart commands -----

func restartBackendCmd(c cfg) tea.Cmd {
	return func() tea.Msg {
		program.Send(logMsg{fmt.Sprintf("%s Restarting backend", tagStart)})
		// stop old if present
		procsMu.Lock()
		old := procs["backend"]
		procsMu.Unlock()
		stopProcess(old)
		killPort(c.backendPort)
		backendArgs := backendUvicornCommand(c)
		beCmd := ""
		if _, err := os.Stat(filepath.Join(c.backendDir, ".venv", "bin", "activate")); err == nil {
			beCmd = "source .venv/bin/activate && stdbuf -oL -eL " + backendArgs
		} else {
			beCmd = "stdbuf -oL -eL " + backendArgs
		}
		if _, err := startProcess("backend", beCmd, c.backendDir); err != nil {
			return startedMsg{what: "backend", ok: false, err: err}
		}
		return startedMsg{what: "backend", ok: true}
	}
}

func restartFrontendCmd(c cfg) tea.Cmd {
	return func() tea.Msg {
		program.Send(logMsg{fmt.Sprintf("%s Restarting frontend", tagStart)})
		procsMu.Lock()
		old := procs["frontend"]
		procsMu.Unlock()
		stopProcess(old)
		killPort(c.frontendPort)
		feCmd := "stdbuf -oL -eL npm run dev -- --host --port " + strconv.Itoa(c.frontendPort)
		if _, err := startProcess("frontend", feCmd, c.frontendDir); err != nil {
			return startedMsg{what: "frontend", ok: false, err: err}
		}
		return startedMsg{what: "frontend", ok: true}
	}
}

// ----- View -----

func (m model) View() string {
	if !m.ready {
		splash := lipgloss.NewStyle().Padding(1, 4).Border(lipgloss.RoundedBorder()).BorderForeground(lipgloss.Color("63")).Render(
			fmt.Sprintf("%s  Initializing UI…", m.spinner.View()),
		)
		return lipgloss.Place(m.width, m.height, lipgloss.Center, lipgloss.Center, splash)
	}
	// Optional splash overlay while starting: take over screen with a centered panel
	if m.starting {
		splash := lipgloss.NewStyle().Padding(1, 4).Border(lipgloss.DoubleBorder()).BorderForeground(lipgloss.Color("63")).Render(
			fmt.Sprintf("%s  Starting GordonBot services…\n\n • Backend (Uvicorn)\n • Frontend (Vite)\n • MediaMTX (optional)", m.spinner.View()),
		)
		return lipgloss.Place(m.width, m.height, lipgloss.Center, lipgloss.Center, splash)
	}

	// Header
	hbFw, _ := headerBox.GetFrameSize()
	headerRendered := headerBox.Width(m.width).Render(clipWidth(headerLines(m), max(1, m.width-hbFw)))
	// Body: wrap viewport or pager in body box to ensure exact width match
	bbFw, _ := bodyBox.GetFrameSize()
	bodyRendered := bodyBox.Width(m.width).Render(clipWidth(m.vp.View(), max(1, m.width-bbFw)))
	// Footer
	fbFw, _ := footerBox.GetFrameSize()
	footerRendered := footerBox.Width(m.width).Render(footerContent(max(1, m.width-fbFw), levelNameFromThreshold(m.levelThreshold), m.mouseEnabled))

	base := lipgloss.JoinVertical(lipgloss.Left, headerRendered, bodyRendered, footerRendered)
	if m.showLevelModal {
		modal := m.levelModalView()
		return lipgloss.Place(m.width, m.height, lipgloss.Center, lipgloss.Center, modal)
	}
	return base
}

func headerLines(m model) string {
	status := ""
	switch m.status {
	case "starting":
		status = warnStyle.Render(m.spinner.View() + " Starting…")
	case "restarting":
		status = warnStyle.Render(m.spinner.View() + " Restarting…")
	case "error":
		status = errStyle.Render("Error starting one or more services")
	default:
		status = okStyle.Render("All services running")
	}
	ip := m.ip
	be := fmt.Sprintf("🐍 Backend:  http://%s:%d/api", ip, m.cfg.backendPort)
	ws := fmt.Sprintf("🔌 WS:      ws://%s:%d/ws/control", ip, m.cfg.backendPort)
	mtx := fmt.Sprintf("🎥 MediaMTX: http://%s:%d (HTTP), rtsp://%s:%d (RTSP)", ip, m.cfg.whepHTTPPort, ip, m.cfg.rtspPort)
	fe := lipgloss.NewStyle().Foreground(lipgloss.Color("10")).Bold(true).Render(fmt.Sprintf("🖥️ Frontend: http://%s:%d", ip, m.cfg.frontendPort))

	// Telemetry
	cpu := "—"
	if m.cpuTemp != nil {
		cpu = fmt.Sprintf("%.1f°C", *m.cpuTemp)
	}
	bp := "—"
	if m.battPct != nil {
		bp = fmt.Sprintf("%.0f%%", *m.battPct)
	}
	state := formatBattState(m.battState)

	headerLines := []string{
		titleStyle.Render("🤖 GordonBot Dev Server"),
		status,
		be,
		ws,
		mtx,
		fe,
		fmt.Sprintf("🌡 CPU: %s   🔋 Battery: %s %s", cpu, bp, state),
	}
	return strings.Join(headerLines, "\n")
}

func footerContent(width int, level string, mouseEnabled bool) string {
	// Stylized hotkey bar with emojis; clipped to inner width
	if level == "" {
		level = "INFO"
	}
	selectMode := "copy"
	if mouseEnabled {
		selectMode = "scroll"
	}
	bar := fmt.Sprintf("Q ⛔ Quit   R 🔄 Reload   B 🐍 Restart Backend   F 🖥️ Restart Frontend   P 📜 Toggle Pager   L 🪵 Filter Level   S 🖱 Select:%s   ◼ Level: %s", selectMode, level)
	if width <= 0 {
		width = 80
	}
	return lipgloss.NewStyle().Width(width).MaxWidth(width).Render(bar)
}

func clipWidth(s string, w int) string {
	if w <= 0 {
		return s
	}
	return lipgloss.NewStyle().Width(w).MaxWidth(w).Render(s)
}

// ------- Log level filtering -------

func levelNames() []string { return []string{"DEBUG", "INFO", "WARN", "ERROR", "CRITICAL"} }

func levelValue(name string) int {
	switch strings.ToUpper(name) {
	case "DEBUG":
		return 10
	case "INFO":
		return 20
	case "WARN", "WARNING":
		return 30
	case "ERROR":
		return 40
	case "CRITICAL", "FATAL":
		return 50
	default:
		return 20
	}
}

var reLevel = regexp.MustCompile(`(?i)\b(DEBUG|INFO|WARN|WARNING|ERROR|CRITICAL|FATAL)\b`)

func lineLevel(line string) int {
	if m := reLevel.FindStringSubmatch(line); m != nil {
		return levelValue(m[1])
	}
	return 20
}

func (m *model) filteredLines() []string {
	if m.levelThreshold <= 10 {
		return m.logs
	}
	out := make([]string, 0, len(m.logs))
	for _, ln := range m.logs {
		if lineLevel(ln) >= m.levelThreshold {
			out = append(out, ln)
		}
	}
	return out
}

func (m *model) appendLog(raw string) {
	m.appendLogRendered(raw, colorizeTags(raw))
}

func (m *model) appendStyledLog(raw string, style lipgloss.Style) {
	styled := style.Render(colorizeTags(raw))
	m.appendLogRendered(raw, styled)
}

func (m *model) appendLogRendered(raw, styled string) {
	m.logs = append(m.logs, styled)
	if len(m.logs) > m.maxLines {
		m.logs = m.logs[len(m.logs)-m.maxLines:]
	}
	writeLogLine(raw)
}

func (m *model) updateContent() {
	// Follow the tail if explicitly enabled or if the viewport was already
	// pinned to the bottom when new content arrived.
	shouldFollow := m.tailFollow || m.vp.AtBottom() || m.vp.PastBottom()
	content := strings.Join(m.filteredLines(), "\n")
	m.vp.SetContent(content)
	if shouldFollow {
		m.vp.GotoBottom()
	}
}

func levelIndexFromThreshold(t int) int {
	switch {
	case t <= 10:
		return 0
	case t <= 20:
		return 1
	case t <= 30:
		return 2
	case t <= 40:
		return 3
	default:
		return 4
	}
}

func (m *model) applySelectedLevel() tea.Cmd {
	names := levelNames()
	if m.modalIndex < 0 || m.modalIndex >= len(names) {
		return nil
	}
	selected := levelValue(names[m.modalIndex])
	oldThreshold := m.levelThreshold
	m.levelThreshold = selected
	wantAccess := selected <= 10
	var restartNeeded bool
	if wantAccess != m.cfg.backendAccessLog {
		restartNeeded = true
		m.cfg.backendAccessLog = wantAccess
	}
	m.showLevelModal = false
	if !restartNeeded {
		m.updateContent()
		return nil
	}
	if oldThreshold != selected {
		if wantAccess {
			m.appendLog(fmt.Sprintf("%s Backend access log enabled (DEBUG level)", tagStart))
		} else {
			m.appendLog(fmt.Sprintf("%s Backend access log disabled", tagStart))
		}
	}
	m.status = "restarting"
	m.starting = true
	m.updateContent()
	return restartBackendCmd(m.cfg)
}

func (m model) levelModalView() string {
	names := levelNames()
	var b strings.Builder
	fmt.Fprintf(&b, "%s\n", titleStyle.Render("Filter Logs by Level"))
	fmt.Fprintf(&b, "%s\n\n", faintStyle.Render("Use ↑/↓ or 1..5, Enter to apply, Esc to cancel"))
	for i, name := range names {
		cursor := "  "
		if i == m.modalIndex {
			cursor = "> "
		}
		line := fmt.Sprintf("%s%d. %s", cursor, i+1, name)
		if i == m.modalIndex {
			line = lipgloss.NewStyle().Bold(true).Render(line)
		}
		fmt.Fprintln(&b, line)
	}
	box := lipgloss.NewStyle().Padding(1, 2).Border(lipgloss.RoundedBorder()).BorderForeground(lipgloss.Color("62"))
	innerW := max(24, m.width/2)
	return box.Width(innerW).Render(b.String())
}

func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}

var tagRegex = regexp.MustCompile(`^\[(start|backend|frontend|mediamtx)\]`)

func colorizeTags(line string) string {
	if m := tagRegex.FindStringSubmatch(line); m != nil {
		switch m[1] {
		case "start":
			return strings.Replace(line, "[start]", tagStart, 1)
		case "backend":
			return strings.Replace(line, "[backend]", tagBackend, 1)
		case "frontend":
			return strings.Replace(line, "[frontend]", tagFrontend, 1)
		case "mediamtx":
			return strings.Replace(line, "[mediamtx]", tagMTX, 1)
		}
	}
	return line
}

// levelNameFromThreshold maps numeric threshold to level name
func levelNameFromThreshold(t int) string {
	if t <= 10 {
		return "DEBUG"
	}
	if t <= 20 {
		return "INFO"
	}
	if t <= 30 {
		return "WARN"
	}
	if t <= 40 {
		return "ERROR"
	}
	return "CRITICAL"
}

// ---------- Backend telemetry ----------

func apiBase(c cfg) string {
	return fmt.Sprintf("http://%s:%d/api", c.apiHost, c.backendPort)
}

func fetchCPUTemp(client *http.Client, c cfg) *float64 {
	url := apiBase(c) + "/diag/system"
	req, _ := http.NewRequest("GET", url, nil)
	resp, err := client.Do(req)
	if err != nil {
		return nil
	}
	defer resp.Body.Close()
	if resp.StatusCode != 200 {
		return nil
	}
	var m map[string]any
	if err := json.NewDecoder(resp.Body).Decode(&m); err != nil {
		return nil
	}
	if v, ok := m["cpu_temperature"]; ok {
		switch t := v.(type) {
		case float64:
			return &t
		case int:
			f := float64(t)
			return &f
		}
	}
	return nil
}

func fetchBattery(client *http.Client, c cfg) (*float64, string) {
	url := apiBase(c) + "/battery/status"
	req, _ := http.NewRequest("GET", url, nil)
	resp, err := client.Do(req)
	if err != nil {
		return nil, ""
	}
	defer resp.Body.Close()
	if resp.StatusCode != 200 {
		return nil, ""
	}
	var b struct {
		Percent  float64 `json:"percent"`
		Charging *string `json:"charging"`
	}
	if err := json.NewDecoder(resp.Body).Decode(&b); err != nil {
		return nil, ""
	}
	var st string
	if b.Charging != nil {
		st = *b.Charging
	}
	return &b.Percent, st
}

func formatBattState(s string) string {
	switch strings.ToLower(s) {
	case "fast_charging":
		return "⚡⚡ fast"
	case "charging":
		return "⚡ charging"
	case "discharging":
		return "⤵ discharging"
	case "idle":
		return "idle"
	default:
		return ""
	}
}

// Bubble Tea commands for periodic telemetry polling
func scheduleTelemetry() tea.Cmd {
	return tea.Tick(5*time.Second, func(time.Time) tea.Msg { return telemetryTickMsg{} })
}

func fetchTelemetryCmd(c cfg) tea.Cmd {
	return func() tea.Msg {
		client := &http.Client{Timeout: 2 * time.Second}
		cpu := fetchCPUTemp(client, c)
		pct, state := fetchBattery(client, c)
		return telemetryMsg{cpu: cpu, pct: pct, state: state}
	}
}

// ---------- main ----------

func main() {
	c := loadCfg()
	initLogFile(c)
	defer closeLogFile()
	writeLogLine("[start] GordonMon session started")
	// Pager-like mode is the default (no auto-follow). Override with GORDONMON_PAGER=0 to tail.
	pagerEnv := os.Getenv("GORDONMON_PAGER")
	pagerMode := true
	if pagerEnv != "" && (pagerEnv == "0" || strings.EqualFold(pagerEnv, "false")) {
		pagerMode = false
	}
	m := initialModel(c, pagerMode)
	p := tea.NewProgram(m, tea.WithAltScreen(), tea.WithMouseCellMotion())
	program = p
	// periodic IP updater
	go func() {
		for {
			time.Sleep(5 * time.Second)
			program.Send(ipUpdateMsg{ip: firstNonLoopbackIPv4()})
		}
	}()
	if _, err := p.Run(); err != nil {
		fmt.Fprintln(os.Stderr, "error:", err)
		writeLogLine(fmt.Sprintf("[start] GordonMon session ended with error: %v", err))
		stopAll()
		closeLogFile()
		os.Exit(1)
	}
	stopAll()
	writeLogLine("[start] GordonMon session ended")
}
