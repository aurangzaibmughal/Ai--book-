# GitHub Pages Deployment Guide

## Overview

This guide walks you through deploying your AI Textbook Platform to GitHub Pages with automated CI/CD using GitHub Actions.

## Prerequisites

- GitHub account
- Git installed locally
- Repository pushed to GitHub
- GitHub Pages enabled in repository settings

## Step-by-Step Deployment

### Step 1: Update Configuration with Your GitHub Details

Edit `docusaurus.config.js` and replace the placeholder values:

```javascript
// Replace these values with your actual GitHub details
url: 'https://YOUR-GITHUB-USERNAME.github.io',
baseUrl: '/YOUR-REPO-NAME/',
organizationName: 'YOUR-GITHUB-USERNAME',
projectName: 'YOUR-REPO-NAME',
```

**Example:**
```javascript
url: 'https://johndoe.github.io',
baseUrl: '/ai-book/',
organizationName: 'johndoe',
projectName: 'ai-book',
```

### Step 2: Create GitHub Repository

1. Go to [GitHub](https://github.com) and sign in
2. Click the "+" icon → "New repository"
3. Repository name: `ai-book` (or your preferred name)
4. Description: "AI Textbook Platform - Interactive learning with Docusaurus"
5. Visibility: **Public** (required for free GitHub Pages)
6. **Do NOT** initialize with README, .gitignore, or license (we already have these)
7. Click "Create repository"

### Step 3: Push Your Code to GitHub

```bash
# Navigate to your project directory
cd "C:\Users\HAJI LAPTOP KARACHI\Desktop\ai-book"

# Initialize git (if not already done)
git init

# Add all files
git add .

# Create initial commit
git commit -m "Initial commit: Docusaurus platform with Chapter 1"

# Add remote repository (replace with your actual URL)
git remote add origin https://github.com/YOUR-USERNAME/ai-book.git

# Push to GitHub
git branch -M main
git push -u origin main
```

### Step 4: Enable GitHub Pages

1. Go to your repository on GitHub
2. Click **Settings** tab
3. Scroll down to **Pages** section (left sidebar)
4. Under "Source":
   - Select: **Deploy from a branch**
   - Branch: **gh-pages**
   - Folder: **/ (root)**
5. Click **Save**

### Step 5: Verify Deployment

1. Go to the **Actions** tab in your repository
2. You should see a workflow run called "Deploy to GitHub Pages"
3. Click on it to see the progress
4. Wait for the workflow to complete (usually 2-5 minutes)
5. Once complete, visit: `https://YOUR-USERNAME.github.io/ai-book/`

## Deployment Workflow

The GitHub Actions workflow (`.github/workflows/deploy.yml`) automatically:

1. **Triggers** on every push to the `main` branch
2. **Checks out** your code
3. **Sets up** Node.js 18
4. **Installs** dependencies with `npm ci`
5. **Builds** the site with `npm run build`
6. **Deploys** to the `gh-pages` branch
7. **GitHub Pages** serves the site from `gh-pages` branch

## Monitoring Deployments

### Check Deployment Status

**Via GitHub Actions:**
1. Go to **Actions** tab
2. View recent workflow runs
3. Green checkmark = successful deployment
4. Red X = failed deployment (click to see logs)

**Via GitHub Pages:**
1. Go to **Settings** → **Pages**
2. See "Your site is live at..." message
3. Click the URL to visit your site

### Deployment Timeline

- **Commit & Push**: Instant
- **GitHub Actions Build**: 2-3 minutes
- **GitHub Pages Update**: 1-2 minutes
- **Total**: ~5 minutes from push to live site

## Troubleshooting

### Issue 1: Workflow Fails with "npm ci" Error

**Cause**: Dependency installation issues

**Solution**:
1. Delete `package-lock.json` locally
2. Run `npm install` to regenerate it
3. Commit and push the new `package-lock.json`

### Issue 2: Site Shows 404 Error

**Cause**: Incorrect `baseUrl` in configuration

**Solution**:
1. Check `docusaurus.config.js`:
   ```javascript
   baseUrl: '/ai-book/',  // Must match repository name
   ```
2. Ensure it ends with `/`
3. Commit, push, and wait for redeployment

### Issue 3: GitHub Pages Not Enabled

**Cause**: Repository settings not configured

**Solution**:
1. Go to Settings → Pages
2. Select source: "Deploy from a branch"
3. Select branch: `gh-pages`
4. Save and wait for deployment

### Issue 4: Workflow Doesn't Trigger

**Cause**: Workflow file not in correct location

**Solution**:
1. Verify file exists at: `.github/workflows/deploy.yml`
2. Check file is committed and pushed
3. Verify branch name is `main` (not `master`)

### Issue 5: Build Succeeds but Site Doesn't Update

**Cause**: GitHub Pages cache

**Solution**:
1. Wait 5-10 minutes for cache to clear
2. Hard refresh browser (Ctrl+Shift+R)
3. Try incognito/private browsing mode

## Custom Domain (Optional)

### Step 1: Add CNAME File

Create `static/CNAME` with your domain:

```
yourdomain.com
```

### Step 2: Configure DNS

Add these DNS records with your domain provider:

**For apex domain (yourdomain.com):**
```
A     @     185.199.108.153
A     @     185.199.109.153
A     @     185.199.110.153
A     @     185.199.111.153
```

**For subdomain (www.yourdomain.com):**
```
CNAME www   YOUR-USERNAME.github.io
```

### Step 3: Update Configuration

Edit `docusaurus.config.js`:

```javascript
url: 'https://yourdomain.com',
baseUrl: '/',
```

### Step 4: Enable HTTPS

1. Go to Settings → Pages
2. Check "Enforce HTTPS"
3. Wait for SSL certificate to provision (can take up to 24 hours)

## Manual Deployment (Alternative)

If you prefer manual deployment without GitHub Actions:

```bash
# Set your GitHub username
GIT_USER=YOUR-USERNAME npm run deploy
```

This will:
1. Build the site
2. Push to `gh-pages` branch
3. Trigger GitHub Pages deployment

## Deployment Checklist

Before deploying, verify:

- [ ] `docusaurus.config.js` has correct URL and baseUrl
- [ ] All content files are committed
- [ ] `package.json` and `package-lock.json` are committed
- [ ] `.github/workflows/deploy.yml` exists and is committed
- [ ] Repository is public (for free GitHub Pages)
- [ ] No sensitive data in code (check .gitignore)

After deploying, verify:

- [ ] GitHub Actions workflow completes successfully
- [ ] Site is accessible at GitHub Pages URL
- [ ] All pages load without 404 errors
- [ ] Navigation works correctly
- [ ] Search functionality works
- [ ] Mermaid diagrams render
- [ ] Responsive design works on mobile
- [ ] Dark mode toggle works

## Continuous Deployment

Once set up, every push to `main` automatically deploys:

```bash
# Make changes to content
vim docs/chapters/chapter-02/index.md

# Commit changes
git add .
git commit -m "Add Chapter 2: Machine Learning"

# Push to GitHub (triggers automatic deployment)
git push origin main

# Wait 5 minutes, then visit your site to see updates
```

## Deployment Best Practices

1. **Test locally first**: Always run `npm start` and verify changes before pushing
2. **Use feature branches**: Create branches for major changes, merge to main when ready
3. **Write clear commit messages**: Helps track what changed in each deployment
4. **Monitor Actions tab**: Check for failed deployments and fix issues promptly
5. **Keep dependencies updated**: Regularly update packages for security and features

## Performance Optimization

### Enable Caching

The workflow already includes npm caching:

```yaml
- name: Setup Node.js
  uses: actions/setup-node@v4
  with:
    node-version: 18
    cache: npm  # Caches node_modules for faster builds
```

### Optimize Build Time

- Keep dependencies minimal
- Use `npm ci` instead of `npm install` (faster, more reliable)
- Enable GitHub Actions caching

### Monitor Build Times

- Check Actions tab for build duration
- Target: &lt;3 minutes for typical builds
- If builds are slow, review dependencies and build process

## Security Considerations

1. **Never commit secrets**: Use GitHub Secrets for sensitive data
2. **Review dependencies**: Regularly audit for vulnerabilities
3. **Use HTTPS**: Always enable "Enforce HTTPS" in GitHub Pages settings
4. **Keep repository public**: Required for free GitHub Pages (or upgrade to Pro)
5. **Review workflow permissions**: Workflow has `contents: write` permission (required for deployment)

## Support

- **GitHub Pages Docs**: https://docs.github.com/en/pages
- **GitHub Actions Docs**: https://docs.github.com/en/actions
- **Docusaurus Deployment**: https://docusaurus.io/docs/deployment

---

**Ready to deploy?** Follow the steps above to get your AI Textbook Platform live on GitHub Pages!
