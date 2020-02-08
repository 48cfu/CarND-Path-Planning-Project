### author @48cfu

## If system is not yet configured
Navigate to installation folder using PowerShell and then
- docker images
- docker system prune
- docker pull udacity/controls_kit:latest

## After setting up the system run like
Navidate to project folder. With src/, installation/ data/ etc..
- docker run -it -p 4567:4567 -v ${pwd}:/work udacity/controls_kit:latest

## The main program can be built and run by doing the following from the project top directory:
- In the terminal execute ./clean.shto make sure you don't have old files in the directory.
- In the terminal execute ./build.sh to build the project.
- In the terminal execute ./run.sh to execute your solution.
