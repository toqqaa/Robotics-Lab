# Author :Toqa 

# Docker Guide

This guide provides step-by-step instructions on how to work with Docker, including pulling images, building containers, running containers, committing changes, opening another terminal, viewing images, listing running containers, and using GUI applications.


---

## Prerequisites

Before you begin, ensure you have the following installed:

* Docker: [Install Docker](https://docs.docker.com/get-docker/)

---

## Pull a Docker Image

To pull a Docker image from Docker Hub, use the following command:

```bash
docker pull <image_name>:<tag>
```

Example:

```bash
docker pull ubuntu:latest
```

This command downloads the latest Ubuntu image from Docker Hub.

---

## Build a Docker Image

To build a Docker image from a `Dockerfile`, navigate to the directory containing the `Dockerfile` and run:


```bash
docker build -t <image_name>:<tag> .
```

Example:

```bash
docker build -t my_custom_image:1.0 .
```

This command builds an image with the tag `my_custom_image:1.0`.

---

## Run a Docker Container

To run a Docker container from an image, use the following command:

```bash
docker run -it --name <container_name> <image_name>:<tag>
```

Example:

```bash
docker run -it --name my_container ubuntu:latest
```

* `-it`: Runs the container in interactive mode with a terminal.
* `--name`: Assigns a name to the container.

---

## Commit Changes to a Docker Image

If you make changes to a running container and want to save those changes as a new image, use the following command:


```bash
docker commit <container_name> <new_image_name>:<tag>
```

Example:


```bash
docker commit my_container my_custom_image:1.1
```

This command creates a new image with the changes made in the container.

---

## Open Another Terminal in a Running Container

To open another terminal session in a running container, use the following command:

```bash
docker exec -it <container_name> /bin/bash
```

Example:

```bash
docker exec -it my_container /bin/bash
```

* `exec`: Executes a command in a running container.
* `/bin/bash`: Opens a Bash shell in the container.

---

## View Docker Images

To list all Docker images on your system, use the following command:

```bash
docker images
```

Example Output:

```bash
REPOSITORY          TAG       IMAGE ID       CREATED        SIZE
ubuntu              latest    123456789abc   2 weeks ago    72.8MB
my_custom_image     1.0       987654321def   1 hour ago     85MB
```

This command shows all the images available locally, along with their tags, image IDs, creation dates, and sizes.

---

## List Running Containers

To list all running Docker containers, use the following command:


```bash
docker ps
```

To list all containers (both running and stopped), use:


```bash
docker ps -a
```

Example Output:

Copy

```
CONTAINER ID   IMAGE           COMMAND       CREATED        STATUS        PORTS     NAMES
abc123456789   ubuntu:latest   "/bin/bash"   5 minutes ago  Up 5 minutes           my_container
```

* `CONTAINER ID`: Unique ID for the container.
* `IMAGE`: The image used to create the container.
* `STATUS`: The current status of the container (e.g., running, exited).

---

## Use GUI Applications with Docker

To run GUI applications inside a Docker container, you need to connect the container to your host's display. Here’s how:

1. **Allow Docker to Access the Host's Display** :
   On Linux, run the following command to allow Docker to access the X server:

```bash
   xhost +local:docker
```

1. **Run the Container with GUI Support** :
   Use the following command to run a container with GUI support:

```bash
   docker run -it --name <container_name> \
              -e DISPLAY=$DISPLAY \
              -v /tmp/.X11-unix:/tmp/.X11-unix \
              <image_name>:<tag>
```

   Example:


```bash
   docker run -it --name gui_container \
              -e DISPLAY=$DISPLAY \
              -v /tmp/.X11-unix:/tmp/.X11-unix \
              ubuntu:latest
```

1. **Install a GUI Application** :
   Inside the container, install a GUI application (e.g., `gedit`):
   bash

   Copy

```
   apt update && apt install -y gedit
```

1. **Run the GUI Application** :
   Start the GUI application from the container's terminal:

```
   gedit
```

   The application window should appear on your host machine.

---

## Use GUI Applications with Docker (Advanced Configuration)

To run GUI applications inside a Docker container, you need to configure the container to communicate with the host's display server (X11). Below is a detailed explanation of the options used in the Docker command:

### Command Structure

```
docker run -it --name=<container_name> \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    <image_name>:<tag>
```

### Explanation of Options

#### 1. `--env="DISPLAY=$DISPLAY"`

* **Purpose** : Connects the container to the host's display.
* **Details** : The `DISPLAY` environment variable tells GUI applications where to render their windows. By passing the host's `DISPLAY` value (`$DISPLAY`) to the container, the container can display windows on the host's screen.
* **Example** : If the host's display is `:0`, the container will also use `:0`.

#### 2. `--env="QT_X11_NO_MITSHM=1"`

* **Purpose** : Disables the use of MIT-SHM (MIT Shared Memory) for Qt-based applications.
* **Details** : Some GUI applications, especially those built with the Qt framework, may have issues with shared memory when running inside a container. This environment variable prevents those issues.
* **Example** : Useful for running applications like `QtCreator` or other Qt-based tools.

#### 3. `--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"`

* **Purpose** : Mounts the host's X11 Unix socket into the container.
* **Details** : The X11 server on Linux uses Unix sockets located in `/tmp/.X11-unix` to communicate with GUI applications. By mounting this directory into the container, the container can communicate with the host's X11 server to display GUI windows.
* **Example** : Required for any GUI application to work inside the container.

#### 4. `--env="XAUTHORITY=$XAUTH"`

* **Purpose** : Sets the `XAUTHORITY` environment variable inside the container.
* **Details** : The `XAUTHORITY` variable points to the X11 authentication file, which is used to authorize the container to access the host's display. This ensures secure communication with the X11 server.
* **Example** : If `XAUTH` is set to `/home/user/.Xauthority` on the host, the container will use the same file.

#### 5. `--volume="$XAUTH:$XAUTH"`

* **Purpose** : Mounts the X11 authentication file into the container.
* **Details** : The X11 authentication file (usually located at `~/.Xauthority`) is required for the container to authenticate with the host's X11 server. Mounting this file ensures the container has the necessary permissions.
* **Example** : If `XAUTH` is `/home/user/.Xauthority`, this mounts the file into the container at the same path.

#### 6. `--net=host`

* **Purpose** : Connects the container to the host's network stack.
* **Details** : Instead of using Docker's internal networking, the container shares the host's network interfaces. This simplifies networking for GUI applications or other services that need direct access to the host's network.
* **Example** : Useful for applications that need to communicate with services running on the host or other devices on the same network.

#### 7. `--privileged`

* **Purpose** : Gives the container elevated privileges.
* **Details** : A privileged container has access to all devices on the host and can perform operations that are normally restricted. This is useful for advanced use cases but should be used with caution as it can pose security risks.
* **Example** : Required for certain tasks like accessing hardware devices or modifying kernel parameters.

---

### Example Command

Here’s an example command to run a GUI application (e.g., `gedit`) in a Docker container:


```
docker run -it --name=my_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    ubuntu:latest
```

---



## Additional Resources

* [Docker Documentation](https://docs.docker.com/)
* [Docker Hub](https://hub.docker.com/)

---

## Troubleshooting

If you encounter any issues, ensure Docker is running and you have the necessary permissions. You can check the status of Docker with:

bash

Copy

```
docker --version
```

If you need further assistance, refer to the [Docker documentation](https://docs.docker.com/).

---

Happy Dockering! 🐳
