import http.server
import socketserver
import json

PORT = 8000

class MyRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_POST(self):
        if self.path == '/save':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            form_data = json.loads(post_data.decode('utf-8'))

            # Save data to JSON file
            with open('data.json', 'w') as f:
                json.dump(form_data, f)

            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'Data saved successfully.')
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b'Not Found')

Handler = MyRequestHandler

with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print("Server started at localhost:" + str(PORT))
    httpd.serve_forever()