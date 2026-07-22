
# EE 446 TinyML (SPR26)

Repository for coursework, labs, assignments, and projects completed as part of **EE 446: Tiny Machine Learning (TinyML)** during Spring 2026. This repository contains hands-on exercises covering machine learning model development, optimization, and deployment on resource-constrained embedded devices. 

## Overview

TinyML focuses on deploying machine learning models directly on low-power microcontrollers and edge devices. Throughout this course, topics include:

* Embedded machine learning
* TensorFlow Lite for Microcontrollers
* Quantization and model compression
* Knowledge distillation
* Model pruning
* TinyML deployment workflows
* Human activity recognition
* Gesture recognition
* Anomaly detection
* Ensemble learning for edge AI

The repository serves as a collection of lab submissions, homework assignments, trained models, reports, and deployment code developed throughout the quarter. 

Example content includes:

* **Lab 3** – Quantization, pruning, and knowledge distillation experiments
* **Lab 5** – TensorFlow Lite deployment and Arduino integration
* **Lab 6** – TinyML anomaly detection
* **Lab 7** – Magic Wand gesture dataset training
* **Lab 8** – Tiny ensemble learning and model optimization
* **Lab 9** – ASL gesture recognition using TinyML models
***

## Technologies Used

* Python
* Jupyter Notebooks
* TensorFlow
* TensorFlow Lite (TFLite)
* TensorFlow Lite for Microcontrollers
* Arduino
* C/C++
* Edge AI / Embedded Systems

***

## Model Optimization Techniques

Several TinyML optimization methods are explored throughout the course:

* Post-Training Quantization (PTQ)
* Quantization-Aware Training (QAT)
* Structured and Sparse Pruning
* Knowledge Distillation
* Model Compression for Edge Deployment

Resulting TFLite models are included for comparison across optimization approaches.

***

## Getting Started

### Clone the Repository

```bash
git clone https://github.com/papoochu/EE-446-TinyML.git
cd EE-446-TinyML
```

### Open Notebooks

Launch Jupyter Notebook or Google Colab and open any `.ipynb` file:

```bash
jupyter notebook
```

### Deploy to Hardware

Arduino sketches (`.ino`) and TensorFlow Lite models (`.tflite`) can be used with supported microcontroller platforms for TinyML inference experiments.

***

## Learning Outcomes

This repository demonstrates:

* Building ML models for edge devices
* Converting models to TensorFlow Lite format
* Optimizing models under memory and latency constraints
* Deploying inference pipelines on embedded hardware
* Evaluating accuracy versus efficiency trade-offs

***

## Author

**Ananya Unnikrishnan**  
EE 446 – TinyML  
Spring 2026

***

## License

This repository is intended for educational purposes and coursework documentation.
