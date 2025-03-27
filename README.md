## Running the project
1. Open the run_med_mon.sh script and update {ABSOLUTE_PATH_TO_MEDMON_DIR} to an absolute directory path to the med_mon repository on your linux machine (ex. /home/my_user/repos/med_mon)
2. From the terminal, build the docker image with the build_docker_image.sh script located at the root of the repository.
3. From the terminal, start a container with the built image with the run_med_mon.sh script located at the root of the repository.
4. From the terminal inside the container, launch the med_mon application using the med_mon.launch file located in src/launch_files

## Project description / architecture
TODO


## Troubleshooting
1. If .sh files throw an error '... cannot execute: required file not found', execute the following command for each shell file: dos2unix {scriptFile}
