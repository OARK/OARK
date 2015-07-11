#This file makes the shell not incredibly slow.
#This file must be sourced!

if [ -e .vimrc.fast ]; then
    echo "Changing vim config to make it faster"
  
    mv .vimrc .vimrc.slow
    mv .vim .vim.slow

    mv .vimrc.fast .vimrc
    mv .vim.fast .vim
else
    echo "Your vim is already fast"
fi

if [ "$BASH" '!=' '/usr/bin/bash' ]; then
    bash
else
    echo "Your shell is already fast"
fi
