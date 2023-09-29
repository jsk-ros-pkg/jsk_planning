#!/bin/bash

function repeat(){
  for ((i=0;i<$1;i++)); do
    eval ${*:2}
  done
}

repeat $@

