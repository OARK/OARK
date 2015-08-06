#!/bin/sh
#This file makes the shell slow again
#This file must be sourced!

if [ -e .vimrc.slow ]; then
    echo "Changing vim config to make it slower"
    mv .vimrc .vimrc.fast
    mv .vim .vim.fast

    mv .vimrc.slow .vimrc
    mv .vim.slow .vim


else
    echo "Your vim is already slow"
fi

#Exits the bash session
if [ "${BASH}" '==' '/usr/bin/bash' ]; then
    echo "Exiting bash session"
    exit
else
    echo "Your shell is already slow"
fi

