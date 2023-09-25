# To build
`
cd ardupilot_docker
docker build --ssh default . -f Dockerfile  -t ardupilot_manna`

# To launch :
`sudo docker run --rm -it --network=host --name "ardupilot_sitl_tri" ardupilot_manna:latest`

## WSL2  :
WSL2 don't allow `--network=host` so we need to expose port manually...
`sudo docker run --rm -it -p5760-5790:5760-5790 -p14550:14550 -p9001:9001 --name "ardupilot_sitl_tri" ardupilot_manna:latest`


# Utilities:
There is now a small server on `http://localhost:9001` that allow to manage the SITL instance and access to the STDOUT easily.

## Cmdline
### Status
supervisorctl status

### Interactive
supervisorctl -i
