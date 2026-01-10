---
sidebar_position: 3
---

# ہینڈز آن لیب: Python میں Sentiment Analysis

## مقصد

اس لیب میں، آپ Python استعمال کرتے ہوئے ایک سادہ AI ماڈل بنائیں گے جو متن کا جذباتی تجزیہ (sentiment analysis) کرتا ہے۔ آپ سیکھیں گے کہ کیسے:
- مشین لرننگ لائبریریاں استعمال کریں
- ڈیٹا کو تیار کریں
- ایک ماڈل کی تربیت کریں
- پیشین گوئیاں کریں

## پیشگی ضروریات

- **Python 3.10+** انسٹال ہونا چاہیے
- بنیادی Python کی معلومات
- ضروری لائبریریاں:
  ```bash
  pip install scikit-learn numpy
  ```

## قدم 1: لائبریریاں Import کریں

سب سے پہلے، ہمیں ضروری Python لائبریریاں import کرنی ہوں گی۔

```python
# ضروری لائبریریاں import کریں
from sklearn.feature_extraction.text import CountVectorizer
from sklearn.naive_bayes import MultinomialNB
from sklearn.model_selection import train_test_split
import numpy as np

print("لائبریریاں کامیابی سے import ہو گئیں!")
```

**وضاحت**:
- `CountVectorizer`: متن کو نمبروں میں تبدیل کرتا ہے
- `MultinomialNB`: Naive Bayes classifier (ایک سادہ لیکن مؤثر الگورتھم)
- `train_test_split`: ڈیٹا کو تربیت اور ٹیسٹ سیٹس میں تقسیم کرتا ہے

## قدم 2: تربیتی ڈیٹا تیار کریں

اب ہم کچھ نمونہ ڈیٹا بنائیں گے جس میں جملے اور ان کے جذبات (مثبت یا منفی) شامل ہیں۔

```python
# تربیتی ڈیٹا: جملے اور ان کے لیبلز
texts = [
    "I love this product, it's amazing!",
    "This is the worst experience ever.",
    "Absolutely fantastic, highly recommend!",
    "Terrible quality, very disappointed.",
    "Great service and fast delivery!",
    "Not worth the money, poor quality.",
    "Excellent! Exceeded my expectations.",
    "Horrible, would not buy again.",
    "Very satisfied with my purchase.",
    "Complete waste of time and money."
]

# لیبلز: 1 = مثبت (positive), 0 = منفی (negative)
labels = [1, 0, 1, 0, 1, 0, 1, 0, 1, 0]

print(f"کل نمونے: {len(texts)}")
print(f"مثبت نمونے: {sum(labels)}")
print(f"منفی نمونے: {len(labels) - sum(labels)}")
```

**وضاحت**:
- `texts`: یہ ہمارے تربیتی جملے ہیں
- `labels`: ہر جملے کا جذبہ (1 = مثبت، 0 = منفی)
- ہمارے پاس 5 مثبت اور 5 منفی نمونے ہیں (متوازن ڈیٹا سیٹ)

## قدم 3: متن کو نمبروں میں تبدیل کریں

مشین لرننگ ماڈلز نمبروں کے ساتھ کام کرتے ہیں، اس لیے ہمیں متن کو نمبری خصوصیات میں تبدیل کرنا ہوگا۔

```python
# Vectorizer بنائیں
vectorizer = CountVectorizer()

# متن کو نمبری خصوصیات میں تبدیل کریں
X = vectorizer.fit_transform(texts)

print(f"خصوصیات کی تعداد: {X.shape[1]}")
print(f"الفاظ کی فہرست: {vectorizer.get_feature_names_out()[:10]}")
```

**وضاحت**:
- `CountVectorizer` ہر منفرد لفظ کو ایک خصوصیت میں تبدیل کرتا ہے
- `fit_transform()` الفاظ کی فہرست بناتا ہے اور ڈیٹا کو تبدیل کرتا ہے
- نتیجہ ایک matrix ہے جہاں ہر قطار ایک جملہ ہے اور ہر کالم ایک لفظ

## قدم 4: ڈیٹا کو تقسیم کریں

ہم اپنے ڈیٹا کو تربیت اور ٹیسٹ سیٹس میں تقسیم کریں گے۔

```python
# ڈیٹا کو 80% تربیت اور 20% ٹیسٹ میں تقسیم کریں
X_train, X_test, y_train, y_test = train_test_split(
    X, labels, test_size=0.2, random_state=42
)

print(f"تربیتی نمونے: {X_train.shape[0]}")
print(f"ٹیسٹ نمونے: {X_test.shape[0]}")
```

**وضاحت**:
- `test_size=0.2`: 20% ڈیٹا ٹیسٹنگ کے لیے
- `random_state=42`: نتائج کو قابل تکرار بناتا ہے
- یہ ماڈل کی کارکردگی کو جانچنے کے لیے اہم ہے

## قدم 5: ماڈل کی تربیت کریں

اب ہم اپنے Naive Bayes classifier کی تربیت کریں گے۔

```python
# ماڈل بنائیں
model = MultinomialNB()

# ماڈل کی تربیت کریں
model.fit(X_train, y_train)

print("ماڈل کی تربیت مکمل!")

# تربیتی ڈیٹا پر درستگی
train_accuracy = model.score(X_train, y_train)
print(f"تربیتی درستگی: {train_accuracy * 100:.2f}%")
```

**وضاحت**:
- `MultinomialNB()`: Naive Bayes classifier بناتا ہے
- `fit()`: ماڈل کو تربیتی ڈیٹا پر تربیت دیتا ہے
- `score()`: ماڈل کی درستگی کا حساب لگاتا ہے

## قدم 6: پیشین گوئیاں کریں

اب ہم نئے جملوں پر اپنے ماڈل کو آزما سکتے ہیں!

```python
# نئے جملے جن کا تجزیہ کرنا ہے
new_texts = [
    "This is wonderful!",
    "I hate this so much.",
    "Pretty good overall.",
    "Absolutely terrible experience."
]

# جملوں کو تبدیل کریں
new_X = vectorizer.transform(new_texts)

# پیشین گوئیاں کریں
predictions = model.predict(new_X)

# نتائج دکھائیں
print("\n=== پیشین گوئیاں ===")
for text, pred in zip(new_texts, predictions):
    sentiment = "مثبت 😊" if pred == 1 else "منفی 😞"
    print(f"جملہ: '{text}'")
    print(f"جذبہ: {sentiment}\n")
```

**وضاحت**:
- `transform()`: نئے جملوں کو اسی طرح تبدیل کرتا ہے جیسے تربیتی ڈیٹا
- `predict()`: ہر جملے کے لیے جذبہ کی پیشین گوئی کرتا ہے
- نتائج 0 (منفی) یا 1 (مثبت) ہوں گے

## متوقع نتیجہ

جب آپ یہ کوڈ چلائیں گے، آپ کو کچھ اس طرح کا نتیجہ ملے گا:

```
لائبریریاں کامیابی سے import ہو گئیں!
کل نمونے: 10
مثبت نمونے: 5
منفی نمونے: 5
خصوصیات کی تعداد: 42
تربیتی نمونے: 8
ٹیسٹ نمونے: 2
ماڈل کی تربیت مکمل!
تربیتی درستگی: 100.00%

=== پیشین گوئیاں ===
جملہ: 'This is wonderful!'
جذبہ: مثبت 😊

جملہ: 'I hate this so much.'
جذبہ: منفی 😞

جملہ: 'Pretty good overall.'
جذبہ: مثبت 😊

جملہ: 'Absolutely terrible experience.'
جذبہ: منفی 😞
```

## چیلنجز

اب جب آپ نے بنیادی sentiment analysis ماڈل بنا لیا ہے، ان چیلنجز کو آزمائیں:

### چیلنج 1: مزید ڈیٹا شامل کریں
`texts` اور `labels` لسٹوں میں 10 مزید جملے شامل کریں اور دیکھیں کہ درستگی کیسے بدلتی ہے۔

### چیلنج 2: اعتماد کی سکور
`predict_proba()` استعمال کرتے ہوئے ہر پیشین گوئی کے لیے اعتماد کی سکور دکھائیں:

```python
probabilities = model.predict_proba(new_X)
for text, prob in zip(new_texts, probabilities):
    print(f"'{text}': {prob[1]*100:.1f}% مثبت")
```

### چیلنج 3: اردو متن
اردو جملوں کے ساتھ ایک نیا ماڈل بنائیں اور دیکھیں کہ کیا یہ کام کرتا ہے!

## مسائل کا حل

### مسئلہ: Import Error

**خرابی**: `ModuleNotFoundError: No module named 'sklearn'`

**حل**: لائبریری انسٹال کریں:
```bash
pip install scikit-learn
```

### مسئلہ: کم درستگی

**خرابی**: ماڈل کی درستگی بہت کم ہے

**حل**:
- مزید تربیتی ڈیٹا شامل کریں
- ڈیٹا کو متوازن رکھیں (برابر مثبت اور منفی نمونے)
- مختلف الگورتھم آزمائیں

## مکمل کوڈ

یہاں ایک جگہ پر مکمل کوڈ ہے:

```python
# ضروری لائبریریاں import کریں
from sklearn.feature_extraction.text import CountVectorizer
from sklearn.naive_bayes import MultinomialNB
from sklearn.model_selection import train_test_split

# تربیتی ڈیٹا
texts = [
    "I love this product, it's amazing!",
    "This is the worst experience ever.",
    "Absolutely fantastic, highly recommend!",
    "Terrible quality, very disappointed.",
    "Great service and fast delivery!",
    "Not worth the money, poor quality.",
    "Excellent! Exceeded my expectations.",
    "Horrible, would not buy again.",
    "Very satisfied with my purchase.",
    "Complete waste of time and money."
]

labels = [1, 0, 1, 0, 1, 0, 1, 0, 1, 0]

# متن کو نمبروں میں تبدیل کریں
vectorizer = CountVectorizer()
X = vectorizer.fit_transform(texts)

# ڈیٹا کو تقسیم کریں
X_train, X_test, y_train, y_test = train_test_split(
    X, labels, test_size=0.2, random_state=42
)

# ماڈل کی تربیت کریں
model = MultinomialNB()
model.fit(X_train, y_train)

print(f"تربیتی درستگی: {model.score(X_train, y_train) * 100:.2f}%")

# نئے جملوں پر پیشین گوئیاں
new_texts = [
    "This is wonderful!",
    "I hate this so much.",
    "Pretty good overall.",
    "Absolutely terrible experience."
]

new_X = vectorizer.transform(new_texts)
predictions = model.predict(new_X)

print("\n=== پیشین گوئیاں ===")
for text, pred in zip(new_texts, predictions):
    sentiment = "مثبت 😊" if pred == 1 else "منفی 😞"
    print(f"'{text}': {sentiment}")
```

---

## اگلا قدم

مبارک ہو! آپ نے اپنا پہلا AI ماڈل بنا لیا ہے۔ اب [مشقیں](./exercises.md) میں جائیں تاکہ اپنی سمجھ کو مضبوط کر سکیں۔
