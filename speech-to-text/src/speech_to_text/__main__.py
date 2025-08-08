import cgi
import logging
import os
import re
import shutil
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, HTTPServer

from faster_whisper import WhisperModel

CHUNKS_DIR = os.getenv(
    "S2T_CHUNKS_DIR", "/tmp/chunks"
)  # dirname where chunks will be saved temporarily
AUDIO_DIR = os.getenv(
    "S2T_AUDIO_DIR", "/tmp"
)  # dirname where the complete audio file will be saved temporarily
AUDIO_PATH = os.path.join(AUDIO_DIR, "audio.wav")

model = WhisperModel("tiny", device="cpu", compute_type="int8")


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        if self.path == "/transcribe":
            # Check for Content-Range header
            content_range = self.headers.get("Content-Range")
            if content_range:
                # Parse Content-Range header (format: bytes start-end/total)
                range_match = re.match(r"bytes (\d+)-(\d+)/(\d+)", content_range)
                if not range_match:
                    self._set_headers(400)
                    self.wfile.write(b"Error: Invalid Content-Range format")
                    return
                start = int(range_match.group(1))
                end = int(range_match.group(2))
                total_size = int(range_match.group(3))
                chunk_size = end - start + 1

                # Download chunk
                file_size = self.download_file(
                    os.path.join(CHUNKS_DIR, f"{start}-{end}")
                )
                if file_size != chunk_size:
                    logging.info("Chunk has wrong size")
                    self.send_response(400)
                    self.end_headers()
                    self.wfile.write(
                        f"Error: Chunk has unexpected size. Expected {chunk_size}, received {file_size}"
                    )
                    return

                # Check if chunk is last
                if end == (total_size - 1):
                    self.combine_chunks(AUDIO_PATH, CHUNKS_DIR)
                    logging.info(f"Combined chunks as {AUDIO_PATH}")
                else:
                    self.send_response(200)
                    self.end_headers()
                    return
            else:  # download complete (not chunked) file
                if self.download_file(AUDIO_PATH) == 0:
                    logging.info("Received empty or no file")
                    self.send_response(400)
                    self.end_headers()
                    self.wfile.write(b"No file uploaded")
                    return

            try:
                # Transcribe using Faster Whisper
                segments, info = model.transcribe(AUDIO_PATH)
                logging.info(
                    "Detected language '%s' with probability %f"
                    % (info.language, info.language_probability)
                )
                text = ""
                for segment in segments:
                    text += segment.text
                text = text.strip()

                # Delete the audio file
                os.unlink(AUDIO_PATH)

                # Send the transcription result
                self.send_response(200)
                self.end_headers()
                self.wfile.write(text.encode("utf-8"))

            except Exception as e:
                # Delete the audio file if created
                if os.path.exists(AUDIO_PATH):
                    os.unlink(AUDIO_PATH)

                self.send_response(500)
                self.end_headers()
                self.wfile.write(
                    f"Error during transcription: {str(e)}".encode("utf-8")
                )
        else:
            self.send_response(404)
            self.end_headers()

    def do_GET(self):
        if self.path == "/health":
            self.send_response(200)
            self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    def log_request(self, code="-", size="-"):
        # Removed /health from logs so Kubernetes healthcheck wouldn't spam
        if self.path == "/health":
            return
        if isinstance(code, HTTPStatus):
            code = code.value
        self.log_message('"%s" %s %s', self.requestline, str(code), str(size))

    def download_file(self, file_path: str):
        """Downloads file from form. Returns number of bytes downloaded."""
        form = cgi.FieldStorage(
            fp=self.rfile, headers=self.headers, environ={"REQUEST_METHOD": "POST"}
        )
        if "file" not in form:
            return 0
        file_item = form["file"]
        file_content = file_item.file.read()

        with open(file_path, "wb") as file:
            file.write(file_content)
        logging.info(f"Downloaded file to {file_path}")
        return os.path.getsize(file_path)

    def combine_chunks(self, final_path, chunks_dir):
        """Combine all chunks into the final file."""
        chunks = sorted(
            [
                os.path.join(chunks_dir, f)
                for f in os.listdir(chunks_dir)
                if os.path.isfile(os.path.join(chunks_dir, f))
            ]
        )

        with open(final_path, "wb") as final_file:
            for chunk_path in chunks:
                with open(chunk_path, "rb") as chunk_file:
                    final_file.write(chunk_file.read())
                os.remove(chunk_path)  # Delete the chunk after combining


def run(server_class=HTTPServer, handler_class=Handler, port=8000):
    server_address = ("", port)
    httpd = server_class(server_address, handler_class)
    logging.info(f"Starting server on port {port}...")
    httpd.serve_forever()


if __name__ == "__main__":
    if os.path.exists(CHUNKS_DIR):
        shutil.rmtree(CHUNKS_DIR)
    os.mkdir(CHUNKS_DIR, mode=0o755)
    logging.basicConfig(
        format="%(asctime)s - %(levelname)s: %(message)s", level=logging.DEBUG
    )
    run()
