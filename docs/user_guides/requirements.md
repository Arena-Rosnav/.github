# Requirements
Arena-Rosnav is currently developed in and for Ubuntu 20.04.

### Git

```bash
sudo apt update && sudo apt install -y git
```

### ROS - Robot Operating System
###### 1. Configure your Ubuntu repositories

```bash
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo add-apt-repository restricted
sudo apt update
```

###### 2. Set up your scources.list

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

###### 3. Set up your keys

```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

###### 4. Installation

```bash
sudo apt update && sudo apt install -y ros-noetic-desktop-full
```

###### 5. Environment Setup

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc # for bash
source ~/.bashrc
```

```bash
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc # for zsh
source ~/.zshrc
```

###### 6. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### Python Poetry

```bash
curl -sSL https://install.python-poetry.org | python3 -
poetry --version
```
Afterwards you should see the version of Poetry in your terminal. If, that is not the case, make sure that the poetry wrapper is correctly included in the PATH variable.
You can do this by executing one of these commands, depending on which shell you are using.
```bash
echo "export PATH=$HOME/.local/bin:$PATH" >> ~/.bashrc # for bash
source ~/.bashrc
```
```bash
echo "export PATH=$HOME/.local/bin:$PATH" >> ~/.zshrc # for zsh
source ~/.zshrc
```
## Recommendations (Optional)

### oh-my-zsh
We recommend using oh-my-zsh. You can set up oh-my-zsh with the following steps.
###### 1. Install zsh
```bash
sudo apt install zsh
```
###### 2. Install curl
```bash
sudo apt install curl
```
###### 3. Install oh-my-zsh
```bash
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)"
```
###### 4. Set up autosuggestions
```bash
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
```
- Open your .zshrc with editor
```bash
nano ~/.zshrc
```
- Scroll down to ```plugins```
- Insert ```zsh-autosuggestions``` into ```plugins=()```.
Your plugins might look like this:
```
plugins=(git zsh-autosuggestions)
```
- Save with Ctrl+S
- Exit with Crtl+X
- Source your .zshrc
```bash
source $HOME/.zshrc
```
###### 5. Install Powerlevel10k Theme (optional)
```bash
git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
```
- In your zshrc, search for ZSH_THEME, set ZSH_THEME="powerlevel10k/powerlevel10k", and follow install wizard.

### Visual Studio Code
We recommend using VSC as your programming environment. You can download VSC from the [official site](https://code.visualstudio.com/download).

## For Windows Users

### WSL2 (Windows-Subsystem for Linux)
Please follow the steps in this [WSL installation guide for Windows 10](https://docs.microsoft.com/en-us/windows/wsl/install-win10) to install WSL2 on your computer.

!!! note 

    You might encounter this problem during installation:

    ```
    Installing, this may take a few minutes...
    WslRegisterDistribution failed with error: 0x80370102
    Error: 0x80370102 The virtual machine could not be started because a required feature is not installed.
    ```

    This problem can be resolved by enabling CPU virtualization in your BIOS. How you can achieve this depends on your hardware.
    [This guide from bleepingcomputer](https://www.bleepingcomputer.com/tutorials/how-to-enable-cpu-virtualization-in-your-computer-bios/) might help you with that.

### Windows-X-Server
To use WSL with graphical programs, an X-server will need to be installed on the Windows 10 system and the DISPLAY variable will need to be set in Bash/Zsh.
One possible program to use is [VcXsrv](https://sourceforge.net/projects/vcxsrv/).

###### 1. Set up DISPLAY variable
- After installing the X-server you need to set the DISPLAY variable in your bash/zsh.
- Use ```nano ~/.bashrc``` or ```nano ~/.zshrc``` and insert the following code on the bottom of the file.

```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

- Save with Ctrl+S and exit via Ctrl+X.

###### 2. Xlaunch Settings
- Start Xlaunch and configure it the following way. In the end the configuration can be saved.
###### 3. Display Settings
- Choose Option: Multiple Windows
- Set Display Number to 0

![image](../images/display_settings.png)

###### 4. Client Settings
- Choose Option: Start no Client

![image](../images/client_settings.png)

###### 5. Extra Settings
- Choose Option: Disable access control

![image](../images/extra_settings.png)

!!! note
    If you encounter problems, you might go to Windows Defender Firewall -> Communication between Applications and Windows Firewall.
    Look for VcXsrv and change the settings to both private and public checked.

    Another problem might occur because of inbound rules from Windows Defender Firewall. Go to Windows Defender Firewall -> Advanced Settings -> Inbound -> Delete every rule for VcXsrv.

### Visual Studio Code plus WSL Extension
We recommend you use Visual Studio Code as your programming environment. Please follow the instructions in this [VS Code with WSL tutorial](https://docs.microsoft.com/en-us/windows/wsl/tutorials/wsl-vscode).
