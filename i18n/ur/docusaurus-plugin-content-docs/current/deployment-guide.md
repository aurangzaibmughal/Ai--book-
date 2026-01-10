---
id: deployment-guide
title: GitHub Pages پر تعیناتی کی گائیڈ
sidebar_label: تعیناتی کی گائیڈ
sidebar_position: 100
description: اپنے Docusaurus سائٹ کو GitHub Pages پر تعینات کرنے کے لیے مکمل گائیڈ
---

# GitHub Pages پر تعیناتی کی گائیڈ

یہ گائیڈ آپ کو اپنی Docusaurus سائٹ کو GitHub Pages پر تعینات کرنے میں مدد کرے گی۔

## پیشگی ضروریات

تعیناتی سے پہلے، یقینی بنائیں کہ آپ کے پاس یہ ہیں:

- ✅ GitHub اکاؤنٹ
- ✅ Git انسٹال ہے
- ✅ Node.js 18+ انسٹال ہے
- ✅ آپ کا پروجیکٹ GitHub repository میں ہے

## قدم 1: Repository کی ترتیبات

### 1.1 Repository بنائیں

اگر آپ نے ابھی تک نہیں کیا:

```bash
git init
git add .
git commit -m "Initial commit"
git branch -M main
git remote add origin https://github.com/your-username/ai-book.git
git push -u origin main
```

### 1.2 GitHub Pages کو فعال کریں

1. اپنی GitHub repository پر جائیں
2. **Settings** → **Pages** پر کلک کریں
3. **Source** کے تحت **GitHub Actions** منتخب کریں

## قدم 2: GitHub Actions Workflow

`.github/workflows/deploy.yml` فائل پہلے سے تشکیل شدہ ہے اور خودکار طور پر:

- ہر `main` branch push پر تعینات کرتی ہے
- سائٹ بناتی ہے
- GitHub Pages پر تعینات کرتی ہے

### Workflow کی تفصیلات

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches:
      - main

permissions:
  contents: read
  pages: write
  id-token: write

jobs:
  deploy:
    environment:
      name: github-pages
      url: ${{ steps.deployment.outputs.page_url }}
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: npm
      - run: npm ci
      - run: npm run build
      - uses: actions/configure-pages@v4
      - uses: actions/upload-pages-artifact@v3
        with:
          path: build
      - id: deployment
        uses: actions/deploy-pages@v4
```

## قدم 3: تعیناتی

### خودکار تعیناتی

صرف اپنی تبدیلیاں push کریں:

```bash
git add .
git commit -m "Update content"
git push origin main
```

GitHub Actions خودکار طور پر:
1. آپ کی سائٹ بنائے گا
2. اسے GitHub Pages پر تعینات کرے گا
3. 2-5 منٹ میں دستیاب ہوگا

### تعیناتی کی حیثیت چیک کریں

1. اپنی repository میں **Actions** ٹیب پر جائیں
2. تازہ ترین workflow run دیکھیں
3. سبز چیک مارک کامیابی کی نشاندہی کرتا ہے

## قدم 4: اپنی سائٹ تک رسائی

تعیناتی کے بعد، آپ کی سائٹ یہاں دستیاب ہوگی:

```
https://your-username.github.io/ai-book/
```

## مقامی طور پر پروڈکشن بلڈ کی جانچ

تعیناتی سے پہلے، مقامی طور پر پروڈکشن بلڈ کی جانچ کریں:

```bash
npm run build
npm run serve
```

یہ `http://localhost:3000/ai-book/` پر ایک مقامی سرور شروع کرے گا۔

## عام مسائل اور حل

### مسئلہ 1: 404 خرابی

**علامات**: سائٹ لوڈ نہیں ہوتی، 404 خرابی دکھاتی ہے

**حل**:
1. `docusaurus.config.js` میں `baseUrl` چیک کریں
2. یقینی بنائیں کہ یہ `/ai-book/` پر سیٹ ہے (آپ کی repository کا نام)
3. اگر repository کا نام `your-username.github.io` ہے، تو `baseUrl: '/'` استعمال کریں

### مسئلہ 2: CSS/JS فائلیں لوڈ نہیں ہوتیں

**علامات**: سائٹ بغیر اسٹائلنگ کے دکھائی دیتی ہے

**حل**:
1. `url` اور `baseUrl` کی ترتیبات چیک کریں
2. یقینی بنائیں کہ `url` `https://your-username.github.io` ہے
3. `baseUrl` repository کے نام سے مماثل ہونا چاہیے

### مسئلہ 3: تعیناتی ناکام ہو جاتی ہے

**علامات**: GitHub Actions workflow سرخ X دکھاتا ہے

**حل**:
1. Actions ٹیب میں خرابی کے لاگز چیک کریں
2. عام مسائل:
   - بلڈ کی خرابیاں (مقامی طور پر `npm run build` چلائیں)
   - اجازات کے مسائل (Repository Settings → Actions → General میں اجازات چیک کریں)
   - Node.js ورژن کی عدم مطابقت

### مسئلہ 4: تبدیلیاں ظاہر نہیں ہوتیں

**علامات**: نئی تبدیلیاں سائٹ پر نظر نہیں آتیں

**حل**:
1. 5 منٹ انتظار کریں (تعیناتی میں وقت لگتا ہے)
2. اپنا براؤزر کیش صاف کریں (Ctrl+Shift+R یا Cmd+Shift+R)
3. Incognito/Private موڈ میں چیک کریں

## اعلی درجے کی ترتیبات

### کسٹم ڈومین

کسٹم ڈومین استعمال کرنے کے لیے:

1. `static/CNAME` فائل بنائیں:
   ```
   yourdomain.com
   ```

2. اپنے DNS فراہم کنندہ میں:
   - `A` ریکارڈ شامل کریں جو GitHub Pages IPs کی طرف اشارہ کرے
   - یا `CNAME` ریکارڈ `your-username.github.io` کی طرف اشارہ کرے

3. Repository Settings → Pages میں کسٹم ڈومین شامل کریں

### ماحول کے متغیرات

اگر آپ کو ماحول کے متغیرات کی ضرورت ہے:

1. Repository Settings → Secrets and variables → Actions پر جائیں
2. **New repository secret** پر کلک کریں
3. اپنے secrets شامل کریں
4. `.github/workflows/deploy.yml` میں ان کا حوالہ دیں:

```yaml
- run: npm run build
  env:
    MY_SECRET: ${{ secrets.MY_SECRET }}
```

## کارکردگی کی اصلاح

### 1. تصاویر کو بہتر بنائیں

```bash
npm install --save-dev @docusaurus/plugin-ideal-image
```

### 2. Compression فعال کریں

GitHub Pages خودکار طور پر gzip compression فراہم کرتا ہے۔

### 3. Caching

GitHub Pages خودکار طور پر static assets کو cache کرتا ہے۔

## نگرانی

### Analytics شامل کریں

`docusaurus.config.js` میں:

```javascript
gtag: {
  trackingID: 'G-XXXXXXXXXX',
  anonymizeIP: true,
},
```

### Uptime Monitoring

استعمال کریں:
- [UptimeRobot](https://uptimerobot.com/)
- [Pingdom](https://www.pingdom.com/)
- [StatusCake](https://www.statuscake.com/)

## بیک اپ اور بحالی

### خودکار بیک اپ

GitHub خودکار طور پر آپ کے کوڈ کا بیک اپ لیتا ہے۔ اضافی بیک اپ کے لیے:

```bash
git clone https://github.com/your-username/ai-book.git backup-folder
```

### پچھلے ورژن پر واپس جانا

```bash
git log  # commit hash تلاش کریں
git revert <commit-hash>
git push origin main
```

## سیکیورٹی

### HTTPS

GitHub Pages خودکار طور پر HTTPS فراہم کرتا ہے۔

### Dependencies کی تازہ کاری

باقاعدگی سے dependencies کو اپ ڈیٹ کریں:

```bash
npm audit
npm update
```

## اگلے قدمات

تعیناتی کے بعد:

1. ✅ اپنی سائٹ کی جانچ کریں
2. ✅ تمام صفحات کام کر رہے ہیں تصدیق کریں
3. ✅ زبان کی تبدیلی کی جانچ کریں
4. ✅ موبائل responsiveness چیک کریں
5. ✅ SEO metadata کی تصدیق کریں

## مدد اور معاونت

اگر آپ کو مسائل کا سامنا ہے:

- 📖 [Docusaurus دستاویزات](https://docusaurus.io/docs/deployment)
- 📖 [GitHub Pages دستاویزات](https://docs.github.com/en/pages)
- 💬 [Docusaurus Discord](https://discord.gg/docusaurus)
- 🐛 [GitHub Issues](https://github.com/facebook/docusaurus/issues)

---

**مبارک ہو!** 🎉 آپ کی سائٹ اب GitHub Pages پر زندہ ہے!
