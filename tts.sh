echo $1

echo "$1" | ./piper --model en_GB-vctk-medium.onnx --output-raw -s $2 | aplay -r 22050 -f S16_LE -t raw -
