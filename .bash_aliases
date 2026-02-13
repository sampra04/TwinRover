alias ll='ls -l'
alias la='ls -la'
alias gs='git status'
alias update='sudo apt update && sudo apt upgrade'
alias rm='rm -i'
alias clean="rm -rf build log install && colcon build && source install/setup.bash && source install/local_setup.bash"

