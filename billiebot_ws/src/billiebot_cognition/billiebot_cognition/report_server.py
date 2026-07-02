#!/usr/bin/env python3
"""FastAPI report server — serves latest daily report on :8080.

Endpoints:
  GET /          — latest HTML report
  GET /latest    — latest Markdown report text
  GET /health    — health check
  GET /reports   — list available reports
"""

import os
import glob

import rclpy
from rclpy.node import Node


class ReportServer(Node):
    """ROS node that launches a FastAPI server for serving reports."""

    def __init__(self):
        super().__init__('report_server')

        self.declare_parameter('report_dir', '/var/lib/billiebot/reports')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)

        self.report_dir = str(self.get_parameter('report_dir').value)
        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)

        os.makedirs(self.report_dir, exist_ok=True)

        # Start server in a background thread
        import threading
        self._server_thread = threading.Thread(
            target=self._run_server, daemon=True
        )
        self._server_thread.start()

        self.get_logger().info(
            f'Report server started on http://{self.host}:{self.port}'
        )

    def _get_latest_report(self) -> str:
        """Find the most recent report markdown file."""
        pattern = os.path.join(self.report_dir, 'report_*.md')
        files = sorted(glob.glob(pattern), reverse=True)
        if not files:
            return '# No reports available yet\n\nBillieBot has not generated any daily reports.'
        with open(files[0], 'r') as f:
            return f.read()

    def _list_reports(self) -> list:
        """List available report files."""
        pattern = os.path.join(self.report_dir, 'report_*.md')
        files = sorted(glob.glob(pattern), reverse=True)
        return [os.path.basename(f) for f in files]

    def _run_server(self):
        """Run the FastAPI/uvicorn server."""
        try:
            from fastapi import FastAPI
            from fastapi.responses import HTMLResponse, PlainTextResponse, JSONResponse
            import uvicorn
        except ImportError:
            self.get_logger().error(
                'fastapi/uvicorn not installed — '
                'pip install fastapi uvicorn'
            )
            return

        app = FastAPI(title='BillieBot Report Server')

        @app.get('/', response_class=HTMLResponse)
        def index():
            md_content = self._get_latest_report()
            try:
                import markdown
                html = markdown.markdown(
                    md_content, extensions=['tables']
                )
            except ImportError:
                html = f'<pre>{md_content}</pre>'
            return f"""
            <html>
            <head>
                <title>BillieBot Daily Report</title>
                <style>
                    body {{ font-family: sans-serif; max-width: 900px;
                           margin: 40px auto; padding: 0 20px; }}
                    table {{ border-collapse: collapse; width: 100%; }}
                    th, td {{ border: 1px solid #ddd; padding: 8px;
                             text-align: left; }}
                    th {{ background-color: #4CAF50; color: white; }}
                </style>
            </head>
            <body>{html}</body>
            </html>
            """

        @app.get('/latest', response_class=PlainTextResponse)
        def latest():
            return self._get_latest_report()

        @app.get('/health', response_class=JSONResponse)
        def health():
            return {'status': 'ok', 'reports': len(self._list_reports())}

        @app.get('/reports', response_class=JSONResponse)
        def reports():
            return {'reports': self._list_reports()}

        uvicorn.run(app, host=self.host, port=self.port, log_level='warning')


def main(args=None):
    rclpy.init(args=args)
    node = ReportServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
