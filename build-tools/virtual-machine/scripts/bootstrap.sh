#!/bin/bash

# Bootstrap the development environment
apt-get update

# Build essential is needed because we might want local machine unit testing.
# Unit testing with Arduino hardware is difficult.
apt-get install -y build-essential git mercurial libcppunit-dev
