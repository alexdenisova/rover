# Speech-to-Text Server

A Speech-to-Text API based on OpenAI Whisper used to transcribe audio for Rover instructions.

To transcribe audio use:
`curl -X POST -F "file=@/path/audio.wav" http://transcribe.alexdenisova.net/transcribe`

or with chunks:
`curl -X POST -H "Content-Range: bytes 0-1048999/2322510" -F "file=@/path/to/chunk" http://transcribe.alexdenisova.net/transcribe`

The server is deployed in a local cluster, which is located in the same network as the Rover.
