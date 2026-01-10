---
sidebar_position: 4
---

# Exercises

Practice what you've learned with these exercises. They're organized by difficulty level.

## Beginner Exercises

### Exercise 1: AI Classification

**Question**: Classify each of the following as Narrow AI, General AI, or Super AI:

a) A chess-playing program that beats world champions
b) A hypothetical AI that can learn any human skill
c) A spam email filter
d) A virtual assistant like Siri or Alexa
e) An AI that surpasses human intelligence in all domains

<details>
<summary>Show Answer</summary>

**Answers**:
- a) Narrow AI - specialized for chess only
- b) General AI - can transfer learning across domains
- c) Narrow AI - specific task of filtering spam
- d) Narrow AI - limited to voice commands and queries
- e) Super AI - exceeds human capabilities

</details>

### Exercise 2: AI vs ML vs DL

**Question**: For each statement, identify whether it describes AI, Machine Learning (ML), or Deep Learning (DL):

a) Uses neural networks with multiple layers
b) Broad field of making machines intelligent
c) Learns from data without explicit programming
d) Requires large amounts of data and computing power
e) Includes rule-based systems and expert systems

<details>
<summary>Show Answer</summary>

**Answers**:
- a) Deep Learning (DL)
- b) Artificial Intelligence (AI)
- c) Machine Learning (ML)
- d) Deep Learning (DL)
- e) Artificial Intelligence (AI)

</details>

### Exercise 3: Real-World Applications

**Question**: Match each AI application with its primary technique:

Applications:
1. Netflix movie recommendations
2. Self-driving cars
3. Language translation
4. Facial recognition

Techniques:
A. Computer Vision
B. Natural Language Processing
C. Reinforcement Learning
D. Collaborative Filtering

<details>
<summary>Show Answer</summary>

**Answers**:
- 1-D: Netflix uses collaborative filtering for recommendations
- 2-C: Self-driving cars use reinforcement learning
- 3-B: Translation uses NLP
- 4-A: Facial recognition uses computer vision

</details>

## Intermediate Exercises

### Exercise 4: Ethical Analysis

**Scenario**: A company develops an AI hiring system that screens job applications. After deployment, they discover the system rejects more female candidates than male candidates with similar qualifications.

**Questions**:
a) What ethical issue does this scenario illustrate?
b) What might have caused this bias?
c) How could the company address this problem?
d) What preventive measures should have been taken?

<details>
<summary>Show Answer</summary>

**Answers**:

a) **Ethical Issue**: Algorithmic bias and discrimination

b) **Possible Causes**:
- Training data reflected historical hiring biases
- Features correlated with gender (e.g., career gaps)
- Lack of diverse representation in training data

c) **Solutions**:
- Audit the training data for bias
- Retrain with balanced, representative data
- Implement fairness constraints in the algorithm
- Add human oversight to the hiring process

d) **Preventive Measures**:
- Diverse development team
- Bias testing before deployment
- Regular audits and monitoring
- Transparent documentation of data sources

</details>

### Exercise 5: Algorithm Selection

**Question**: For each scenario, recommend the most appropriate ML approach (Supervised, Unsupervised, or Reinforcement Learning) and explain why:

a) Predicting house prices based on historical sales data
b) Grouping customers into segments without predefined categories
c) Training a robot to navigate a maze
d) Classifying emails as spam or not spam
e) Discovering patterns in customer purchase behavior

<details>
<summary>Show Answer</summary>

**Answers**:

a) **Supervised Learning** - We have labeled data (historical prices) to learn from

b) **Unsupervised Learning** - No predefined categories; algorithm finds natural groupings

c) **Reinforcement Learning** - Robot learns through trial and error with rewards/penalties

d) **Supervised Learning** - We have labeled examples of spam and non-spam emails

e) **Unsupervised Learning** - Discovering hidden patterns without predefined labels

</details>

### Exercise 6: Code Analysis

**Question**: Review this code snippet and answer the questions:

```python
from sklearn.tree import DecisionTreeClassifier

# Training data
X_train = [[0, 0], [1, 1], [2, 2], [3, 3]]
y_train = [0, 0, 1, 1]

# Create and train model
model = DecisionTreeClassifier()
model.fit(X_train, y_train)

# Make prediction
prediction = model.predict([[1.5, 1.5]])
```

a) What type of learning is this (supervised, unsupervised, reinforcement)?
b) How many features does each data point have?
c) How many classes are in the output?
d) What would you expect the prediction to be for input `[1.5, 1.5]`?

<details>
<summary>Show Answer</summary>

**Answers**:

a) **Supervised Learning** - We have labeled training data (X_train with y_train)

b) **Two features** - Each data point is a 2D coordinate [x, y]

c) **Two classes** - Classes 0 and 1 (binary classification)

d) **Likely class 0** - The input [1.5, 1.5] is closer to the training points labeled as class 0

</details>

## Advanced Exercises

### Exercise 7: System Design

**Challenge**: Design an AI system for a smart home that learns user preferences and automates home settings (temperature, lighting, music).

**Requirements**:
- Describe the data you would collect
- Choose appropriate ML techniques
- Address privacy concerns
- Explain how the system would learn and adapt

**Deliverable**: Write a 1-page system design document.

### Exercise 8: Bias Detection

**Challenge**: You're given a dataset of loan applications with the following features: income, credit score, employment history, zip code, and loan approval status.

**Tasks**:
a) Identify potential sources of bias in this dataset
b) Propose methods to detect bias in a trained model
c) Suggest techniques to mitigate identified biases
d) Design a fairness evaluation framework

### Exercise 9: Research and Present

**Challenge**: Choose one of the following AI breakthroughs and create a 5-minute presentation:

- AlphaGo defeating world champion Lee Sedol
- GPT-3 and large language models
- DALL-E and image generation
- AlphaFold solving protein folding

**Include**:
- Technical approach used
- Significance of the breakthrough
- Ethical implications
- Future applications

## Solutions and Discussion

Detailed solutions for advanced exercises are available in the course discussion forum. We encourage you to:

1. Attempt the exercises independently first
2. Compare your answers with peers
3. Discuss different approaches
4. Ask questions in the forum

---

**Completed the exercises?** Test your knowledge with the [Quiz](./quiz)!
