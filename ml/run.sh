#!/bin/bash

# Script to create ML Report Project structure

# Project Root Directory
PROJECT_ROOT="ML_Report_Project"
mkdir -p $PROJECT_ROOT
cd $PROJECT_ROOT

# Create main files
touch main.tex references.bib README.md

# Create images folder
mkdir -p images
touch images/example_image.png

# Create chapters and their structure
for i in {1..4}; do
    CHAPTER_DIR="chapters/chapter$i"
    mkdir -p $CHAPTER_DIR
    
    # Create section files for each chapter
    touch $CHAPTER_DIR/abstract.tex
    touch $CHAPTER_DIR/introduction.tex
    touch $CHAPTER_DIR/approach.tex
    touch $CHAPTER_DIR/details.tex
    touch $CHAPTER_DIR/experiments.tex
    touch $CHAPTER_DIR/results.tex
    touch $CHAPTER_DIR/conclusions.tex
    
    # Create main chapter file
    cat > $CHAPTER_DIR/chapter${i}_main.tex <<EOL
\section*{Abstract}
\input{abstract.tex}

\section{Introduction}
\input{introduction.tex}

\section{Approach}
\input{approach.tex}

\section{Details}
\input{details.tex}

\section{Experiments}
\input{experiments.tex}

\section{Results}
\input{results.tex}

\section{Conclusions}
\input{conclusions.tex}
EOL
done

# Create main.tex
cat > main.tex <<EOL
\documentclass[12pt,a4paper]{report}

% Packages
\usepackage[utf8]{inputenc}
\usepackage{amsmath, amssymb}
\usepackage{graphicx}
\usepackage{hyperref}
\usepackage{float}
\usepackage{caption}
\usepackage{subcaption}

% Title Page
\title{Machine Learning Report}
\author{Your Name}
\date{\today}

\begin{document}

\maketitle
\tableofcontents

\chapter{Assignment 1}
\input{chapters/chapter1/chapter1_main.tex}

\chapter{Assignment 2}
\input{chapters/chapter2/chapter2_main.tex}

\chapter{Assignment 3}
\input{chapters/chapter3/chapter3_main.tex}

\chapter{Assignment 4}
\input{chapters/chapter4/chapter4_main.tex}

% Bibliography
\bibliographystyle{plain}
\bibliography{references}

\end{document}
EOL

# Create README.md
cat > README.md <<EOL
# Machine Learning Report Project

## Folder Structure
- **chapters/**: Each chapter is a folder containing structured \`.tex\` files.
- **images/**: Store figures and plots here.
- **main.tex**: Main file to compile the entire report.
- **references.bib**: Bibliography file for citations.

## Chapter Structure
Each chapter contains:
- **abstract.tex**: A brief summary.
- **introduction.tex**: Problem statement and state-of-the-art methods.
- **approach.tex**: Description of your approach.
- **details.tex**: Formulas, optimizations, challenges.
- **experiments.tex**: Description of experiments and datasets.
- **results.tex**: Results in tables, graphs, and analysis.
- **conclusions.tex**: Final conclusions and future work.

## How to Compile
1. Open \`main.tex\` in Overleaf.
2. Compile using PDFLaTeX.
3. Ensure all \`chapterX_main.tex\` files and \`references.bib\` are correctly referenced.

## Author
Your Name
EOL

# Final message
echo "ML Report Project structure created successfully!"
