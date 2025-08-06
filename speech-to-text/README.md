# Speech-to-Text Server

A Speech-to-Text API based on OpenAI Whisper used to transcribe audio for Rover instructions.

To transcribe audio use:
`curl -v -X POST -F "file=@/path/audio.wav" http://transcribe.alexdenisova.net/transcribe`

The server is deployed in a local cluster, which is located in the same network as the Rover.
