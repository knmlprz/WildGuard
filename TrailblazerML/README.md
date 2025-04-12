# TrailblazerML
##### Description of the project
This is a project to create an autonomous control rover for the rover “legnedary rover PRz” the goal is to win the
competition one of the tasks is autonomy

---


# What hardware we are using

- Nvidia Jetson xavier nx
- Camera Oak-D-pro-w
- GPS NEO-6M-0U-BLOX

# What software we are using (main libraries)

- ROS2
- DepthAI
- OpenCV
- Open3d

# Docker

---

## _!IMPORTANT_

Add xhost +local:docker to allow GUI support to bashrc file to make it permanent

1. Bash
```bash
  echo 'xhost +local:docker' >> ~/.bashrc
```
2. Zsh
```bash
  sudo xhost +local:docker >> ~/.zshrc
```


## How to build the docker image

```bash
 sudo docker build -t trb_1 .
```

## How to run the docker image with GUI support

### Linux
```bash
    sudo docker run -it --name <GIVE YOUR OWN NAME FOR CONTAINER> --privileged --network=host --ipc=host --env=DISPLAY trb_1
```
Exit after finishing the work
```bash
    exit
```

## After running the docker image

Run the following command to enter your container
```bash
    sudo docker start -ai <GIVE YOUR OWN NAME>
```
**You will be in the same state as you left the container**

## What is inside the docker image

- ROS2
- Gazebo
- Terminator
- Visual studio code
    To start visual studio code
    ```bash
      code .
    ```

# Saving Docker State with `docker commit`

When making modifications within a running Docker container (such as installing new dependencies or making system
changes), you can save these changes as a new Docker image using `docker commit`. This allows you to preserve your
modifications without needing to rebuild the container from scratch.

#### **Steps to Save and Reuse a Modified Docker Container**

1. **Check running containers**
   ```bash
   docker ps -a
   ```
   Find the `CONTAINER ID` or `NAME` of the running container you want to save.

2. **Create a snapshot (which is just another image) of the current container state**
   ```bash
   docker commit <container_id/container_name> <IMAGE NAME YOU WANT TO CREATE>
   ```
   Replace `<container_id>` with the actual ID of your container.

3. **Run a new container from the saved snapshot**
   ```bash
   sudo docker run -it --name <GIVE YOUR OWN NAME> --privileged --network=host --ipc=host --env=DISPLAY <IMAGE NAME YOU WANT TO CREATE>
   ```
#### **Notes:**

- The snapshot (`trb_snapshot`) can be used as a base for further development.
- This method allows you to keep changes without modifying the original `Dockerfile`.
- If you want to make the changes permanent, consider updating the `Dockerfile` and rebuilding the image
  using `docker build`.
- creating snapshot is useful when you want to save the state of the container and use it later on, for example, when
  you want to test something and you don't want to rebuild the image from scratch.

# Workflow

---

## Branches

- **main**
    - main branch, should be always stable - every pull request to this branch should be reviewed by at least 2-3
      persons.
    - merge to this branch can be done only by the project manager.
    - merge to this branch can be done from **dev** branch.
    - every merge means new version of the project.

- **dev**
    - development branch
    - merge to this branch can be done only if all feature tests are passed.
    - merge to this branch can be only done by feature reviewer
    - acceptance criteria for merge to this branch:
        - have all functionality tests passed.
    - merge to this branch means end of milestone.

- **feature**
    - branches for specific features
    - use to develop new features

- **bug**
    - branches for specific bugs
    - use to fix bugs from feature branch

- **test**
    - branches for specific features
    - use only to test specific feature - do not add new features to this branch

```markdown
├── **main**
│ └── **dev**
│ ├── **feature**
│ │ ├── **bug**
│ │ └── **test**          
│ │
│ ├── **feature**
│ │ ├── **bug**
│ │ └── **test**
│ │
```

## Create Branches

- Create a new branch for each feature or bug fix.
- Branch names should be descriptive:
    ```markdown
        [feature/bug/test]/[milestone]/[short-description]
    ```
  Example:
    ```markdown
        feature/ros_introduction/start_with_ros
    ```

## Commit Messages

- Commit messages should be descriptive - **obligatory use
  of [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)**

## Issues

- Issues have to be created with templates provided in the repository
    - Issue template - create issue ticket
    - Bug report - report a bug for specific issue
- **Every issue have to have assignees and reviewer**

## Pull Requests

- Pull requests should be descriptive and have annotation to issue

