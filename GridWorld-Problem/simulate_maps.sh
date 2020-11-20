#!/bin/bash
python3 generalized_gridWorld.py -lr 0.3 -df 0.99  -e 0.1 -t tk1  -s 10,10 -g 9,0 -o map1.txt -ar y -r 500 &
python3 generalized_gridWorld.py -lr 0.3 -df 0.99  -e 0.1 -t tk2  -s 10,10 -g 9,0 -o map2.txt -ar y -r 500 &
python3 generalized_gridWorld.py -lr 0.3 -df 0.99  -e 0.1 -t tk3  -s 10,10 -g 9,0 -o map3.txt -ar y -r 500 &
python3 generalized_gridWorld.py -lr 0.3 -df 0.99  -e 0.1 -t tk4  -s 10,10 -g 9,0 -o map4.txt -ar y -r 500 &
