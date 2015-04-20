# Continuous Integration

Everything that can be automated, should be automated. The following is an
outline of setting up the continuous integration system. At the moment it's a
rough guide and will be developed further.

## Installation

This will require a server, these instructions assume a Linux server, in
particular Debian. These instructions will not outline the installation unless
there is something specific to the CI server that needs to be done.

## Software selection

The only selected software for the Debian install:
    SSH server
    Standard system utilities

## Jenkins installation

Add the package repository of Jenkins to the server, these instruction can be
found at http://pkg.jenkins-ci.org/debian-stable/ if there are any issues.

    wget -q -O - http://pkg.jenkins-ci.org/debian-stable/jenkins-ci.org.key | sudo apt-key add -

Add the following entry into /etc/apt/sources.list
    deb http://pkg.jenkins-ci.org/debian-stable binary/

Update and install.
    sudo apt-get update
    sudo apt-get install jenkins mercurial

Jenkins will be installed and running on port 8080.

Update any Jenkins plugins.

Install the Mercurial plugin.

## Jenkins configuration

    Manage Jenkins->Configure System

At the moment, we just want to link the Kiln repository with Bitbucket, with any commits on the devel branch being pushed to the SEP 17 repository automatically.

Generate the public/private keys for SSH auth:
    sudo su -s /bin/bash jenkins
    ssh-keygen -t rsa -N ""

Add the generated public key to both Bitbucket and Kiln.

Be sure that any SSH keys of the servers there will be connections to are in the known_hosts file.

## Jenkins authentication

Seems to be a bit tricky. It seems you have to use the API token. When in doubt, put the token everywhere. Doesn't work.

## Notes

At the moment the build will fail if there are no differences in any of the target repos.
