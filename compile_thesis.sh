#!/bin/bash

unzip $1 -d TESE

cd TESE

pdflatex main
makeglossaries main
bibtex main
pdflatex main
pdflatex main
mv main.pdf ../
cd ..
rm -r TESE
xdg-open main.pdf
