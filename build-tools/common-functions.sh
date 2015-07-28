#!/bin/bash

# Common functions for the build scripts.
set -euo pipefail
IFS=$'\n\t'

# Colour codes
export COLOUR_SUCCESS='\033[0;32m'
export COLOUR_PROGRESS='\033[0;33m'
export NC='\033[0m' # No Colour

# I want to be sure that operations are done relative to this scripts
# directory, not from current directory someone may have when running
# it.
#
# http://stackoverflow.com/questions/59895/can-a-bash-script-tell-what-directory-its-stored-in
function common::get_script_directory() {
  local SOURCE="${BASH_SOURCE[0]}"
  while [ -h "$SOURCE" ]; do
    # Resolve $SOURCE until the file is no longer a symlink
    local DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
    local SOURCE="$(readlink "$SOURCE")"
    # If $SOURCE was a relative symlink, we need to resolve it relative to
    # the path where the symlink file was located
    [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE"
  done
  local DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

  echo "$DIR"
}
