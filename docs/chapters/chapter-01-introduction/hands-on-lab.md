---
sidebar_position: 3
---

# Hands-On Lab: Build Your First AI Program

## Lab Overview

**Duration**: 45 minutes
**Difficulty**: Beginner
**Prerequisites**: Basic Python knowledge

In this lab, you'll build a simple AI program that uses machine learning to classify text sentiment (positive or negative). This hands-on exercise will help you understand the basic workflow of an AI project.

## What You'll Build

A sentiment analysis program that can determine whether a text review is positive or negative.

**Example**:
- Input: "This product is amazing!" → Output: Positive
- Input: "Terrible experience, very disappointed." → Output: Negative

## Lab Setup

### Step 1: Install Required Libraries

First, ensure you have Python 3.8+ installed. Then install the required libraries:

```bash
pip install scikit-learn numpy
```

### Step 2: Create Your Project File

Create a new file called `sentiment_analyzer.py`:

```python
# sentiment_analyzer.py
import numpy as np
from sklearn.feature_extraction.text import CountVectorizer
from sklearn.naive_bayes import MultinomialNB
from sklearn.model_selection import train_test_split

# Sample training data
training_data = [
    ("I love this product", "positive"),
    ("This is amazing", "positive"),
    ("Great quality", "positive"),
    ("Excellent service", "positive"),
    ("Best purchase ever", "positive"),
    ("Terrible experience", "negative"),
    ("Very disappointed", "negative"),
    ("Waste of money", "negative"),
    ("Poor quality", "negative"),
    ("Would not recommend", "negative"),
]

# Separate features and labels
texts = [text for text, label in training_data]
labels = [label for text, label in training_data]

# Create feature vectors
vectorizer = CountVectorizer()
X = vectorizer.fit_transform(texts)

# Train the model
classifier = MultinomialNB()
classifier.fit(X, labels)

# Function to predict sentiment
def predict_sentiment(text):
    text_vector = vectorizer.transform([text])
    prediction = classifier.predict(text_vector)
    return prediction[0]

# Test the model
if __name__ == "__main__":
    test_texts = [
        "This is wonderful",
        "I hate this",
        "Pretty good product",
        "Awful experience"
    ]

    print("Sentiment Analysis Results:")
    print("-" * 40)
    for text in test_texts:
        sentiment = predict_sentiment(text)
        print(f"Text: '{text}'")
        print(f"Sentiment: {sentiment}")
        print()
```

### Step 3: Run Your Program

Execute the program:

```bash
python sentiment_analyzer.py
```

**Expected Output**:
```
Sentiment Analysis Results:
----------------------------------------
Text: 'This is wonderful'
Sentiment: positive

Text: 'I hate this'
Sentiment: negative

Text: 'Pretty good product'
Sentiment: positive

Text: 'Awful experience'
Sentiment: negative
```

## Understanding the Code

### 1. Data Preparation

```python
training_data = [
    ("I love this product", "positive"),
    ("Terrible experience", "negative"),
    # ... more examples
]
```

**What's happening**: We create labeled training data - text paired with its sentiment label.

### 2. Feature Extraction

```python
vectorizer = CountVectorizer()
X = vectorizer.fit_transform(texts)
```

**What's happening**: The `CountVectorizer` converts text into numerical features (word counts) that the ML algorithm can process.

### 3. Model Training

```python
classifier = MultinomialNB()
classifier.fit(X, labels)
```

**What's happening**: We use a Naive Bayes classifier to learn patterns from the training data.

### 4. Making Predictions

```python
def predict_sentiment(text):
    text_vector = vectorizer.transform([text])
    prediction = classifier.predict(text_vector)
    return prediction[0]
```

**What's happening**: New text is converted to features and fed to the trained model for prediction.

## Lab Challenges

### Challenge 1: Expand the Training Data

Add 10 more training examples (5 positive, 5 negative) to improve accuracy.

**Hint**: Include diverse vocabulary and sentence structures.

### Challenge 2: Add Neutral Sentiment

Modify the program to classify three sentiments: positive, negative, and neutral.

**Steps**:
1. Add neutral examples to training data
2. Update the model to handle three classes
3. Test with neutral sentences

### Challenge 3: Calculate Confidence

Modify the `predict_sentiment` function to return both the prediction and confidence score.

**Hint**: Use `classifier.predict_proba()` instead of `predict()`.

## Common Issues and Solutions

### Issue 1: Import Error

**Error**: `ModuleNotFoundError: No module named 'sklearn'`

**Solution**: Install scikit-learn:
```bash
pip install scikit-learn
```

### Issue 2: Low Accuracy

**Problem**: Model makes incorrect predictions

**Solutions**:
- Add more training data
- Use more diverse vocabulary
- Try different algorithms (e.g., Logistic Regression)

### Issue 3: Unknown Words

**Problem**: Model struggles with words not in training data

**Solution**: Add more training examples or use pre-trained word embeddings.

## Lab Summary

Congratulations! You've built your first AI program. You learned:

✅ How to prepare training data for ML
✅ How to convert text to numerical features
✅ How to train a classification model
✅ How to make predictions with your model

## Next Steps

- Complete the [Exercises](./exercises) to reinforce your learning
- Try the challenges above to extend your program
- Experiment with different ML algorithms

---

**Questions?** Check the [FAQs](./faqs) for common questions about this lab.
