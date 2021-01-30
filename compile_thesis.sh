#!/bin/bash

unzip $1 -d TESE

cd TESE

pdflatex main
makeglossaries main
bibtex main
pdflatex main
pdflatex main
xdg-open main.pdf
