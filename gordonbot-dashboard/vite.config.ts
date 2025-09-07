import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import tailwindcss from '@tailwindcss/vite'
import { fileURLToPath, URL } from 'node:url'

export default defineConfig({
  plugins: [react(), tailwindcss()],
  resolve: {
    alias: {
      '@': fileURLToPath(new URL('./src', import.meta.url)),
    },
  },
  server: {
    host: true,
    port: 5173,
    proxy: {
      // REST API → FastAPI
      '/api': (() => {
        const base = process.env.VITE_API_BASE || 'http://localhost:8000'
        return {
          target: base,
          changeOrigin: true,
        }
      })(),
      // WebSocket → FastAPI
      '/ws': (() => {
        const base = process.env.VITE_API_BASE || 'http://localhost:8000'
        // Derive ws/wss target from http/https base
        let target = base
        try {
          const u = new URL(base)
          const wsProto = u.protocol === 'https:' ? 'wss:' : 'ws:'
          target = `${wsProto}//${u.host}`
        } catch {
          // keep as-is
        }
        return {
          target,
          ws: true,
          changeOrigin: true,
        }
      })(),
    },
  },
})
