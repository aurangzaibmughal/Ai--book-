# Vercel Deployment Guide

## Overview

This guide walks you through deploying your Physical AI & Humanoid Robotics textbook to Vercel with automatic deployments.

## Prerequisites

- GitHub account with your repository
- Vercel account (free tier available at https://vercel.com)
- Git installed locally

## Quick Start

### Option 1: Deploy via Vercel Dashboard (Recommended)

1. **Sign up/Login to Vercel**
   - Go to https://vercel.com
   - Click "Sign Up" or "Login"
   - Choose "Continue with GitHub" for easiest integration

2. **Import Your Repository**
   - Click "Add New..." → "Project"
   - Select "Import Git Repository"
   - Find and select your `ai-book` repository
   - Click "Import"

3. **Configure Project Settings**

   Vercel should auto-detect Docusaurus. Verify these settings:

   - **Framework Preset**: Docusaurus
   - **Build Command**: `npm run build`
   - **Output Directory**: `build`
   - **Install Command**: `npm install`
   - **Node Version**: 18.x (set in Environment Variables if needed)

4. **Deploy**
   - Click "Deploy"
   - Wait 2-3 minutes for the build to complete
   - Your site will be live at: `https://your-project-name.vercel.app`

### Option 2: Deploy via Vercel CLI

```bash
# Install Vercel CLI globally
npm install -g vercel

# Navigate to your project
cd "C:\Users\HAJI LAPTOP KARACHI\Desktop\ai-book"

# Login to Vercel
vercel login

# Deploy (first time - will ask configuration questions)
vercel

# Follow the prompts:
# - Set up and deploy? Yes
# - Which scope? (select your account)
# - Link to existing project? No
# - Project name? (accept default or customize)
# - Directory? ./ (press Enter)
# - Override settings? No

# Deploy to production
vercel --prod
```

## Configuration Files

### vercel.json

Already created with optimal settings:

```json
{
  "buildCommand": "npm run build",
  "outputDirectory": "build",
  "framework": "docusaurus",
  "installCommand": "npm install",
  "devCommand": "npm start",
  "rewrites": [
    {
      "source": "/(.*)",
      "destination": "/index.html"
    }
  ]
}
```

### docusaurus.config.js

Your configuration automatically detects the deployment environment:

```javascript
// Automatically uses correct baseUrl based on environment
// Vercel: baseUrl = '/'
// GitHub Pages: baseUrl = '/Ai--book-/'
url: process.env.VERCEL_URL
  ? `https://${process.env.VERCEL_URL}`
  : process.env.DEPLOYMENT_URL || 'https://aurangzaibmughal.github.io',
baseUrl: (process.env.VERCEL || process.env.VERCEL_ENV || process.env.VERCEL_URL) ? '/' : '/Ai--book-/',
```

**No manual configuration needed** - the site will automatically use the correct baseUrl for Vercel (`/`) or GitHub Pages (`/Ai--book-/`).

## Automatic Deployments

Once connected, Vercel automatically deploys:

- **Production**: Every push to `main` branch
- **Preview**: Every push to other branches and pull requests
- **Instant Rollback**: Revert to any previous deployment with one click

## Environment Variables (Optional)

If you need environment variables:

1. Go to your project in Vercel Dashboard
2. Click "Settings" → "Environment Variables"
3. Add variables for:
   - `NODE_VERSION`: 18
   - Any API keys or secrets (never commit these to Git)

## Custom Domain (Optional)

### Add Your Domain

1. Go to your project in Vercel Dashboard
2. Click "Settings" → "Domains"
3. Enter your domain (e.g., `robotics-textbook.com`)
4. Click "Add"

### Configure DNS

Vercel will provide DNS records. Add these to your domain provider:

**For apex domain (robotics-textbook.com):**
```
A     @     76.76.21.21
```

**For www subdomain:**
```
CNAME www   cname.vercel-dns.com
```

### Update Configuration

Edit `docusaurus.config.js`:

```javascript
url: 'https://robotics-textbook.com',
baseUrl: '/',
```

Commit and push to trigger redeployment.

## Monitoring Deployments

### Vercel Dashboard

1. Go to https://vercel.com/dashboard
2. Select your project
3. View:
   - **Deployments**: All deployment history
   - **Analytics**: Traffic and performance metrics
   - **Logs**: Build and runtime logs
   - **Speed Insights**: Core Web Vitals

### Deployment Status

- **Building**: Site is being compiled
- **Ready**: Deployment successful and live
- **Error**: Build failed (click to see logs)
- **Canceled**: Deployment was canceled

## Troubleshooting

### Build Fails with "npm ci" Error

**Solution**:
```bash
# Delete package-lock.json
rm package-lock.json

# Regenerate with npm install
npm install

# Commit and push
git add package-lock.json
git commit -m "Update package-lock.json"
git push
```

### Build Fails with Memory Error

**Solution**: Add environment variable in Vercel:
- Key: `NODE_OPTIONS`
- Value: `--max-old-space-size=4096`

### Site Shows 404 on Routes

**Solution**: The `vercel.json` rewrites configuration should handle this. If issues persist:

1. Verify `vercel.json` is committed
2. Check `baseUrl: '/'` in `docusaurus.config.js`
3. Redeploy from Vercel Dashboard

### Urdu (RTL) Content Not Displaying Correctly

**Solution**: Ensure these files are committed:
- `src/css/rtl.css`
- All files in `i18n/ur/` directory

### Search Not Working

**Solution**: The local search plugin should work automatically. If issues:

1. Check `@easyops-cn/docusaurus-search-local` is in `package.json`
2. Verify plugin configuration in `docusaurus.config.js`
3. Clear build cache: `npm run clear && npm run build`

## Performance Optimization

### Enable Edge Network

Vercel automatically serves your site from their global Edge Network (no configuration needed).

### Enable Analytics

1. Go to project Settings → Analytics
2. Enable "Web Analytics"
3. View real-time traffic and Core Web Vitals

### Enable Speed Insights

1. Go to project Settings → Speed Insights
2. Enable feature
3. Monitor performance metrics

## Deployment Checklist

**Before First Deployment:**
- [x] `vercel.json` created and committed
- [x] `package.json` has correct build script
- [x] Local build succeeds (`npm run build`)
- [x] All content files committed to Git
- [x] Repository pushed to GitHub
- [ ] Vercel account created
- [ ] Repository imported to Vercel

**After Deployment:**
- [ ] Site accessible at Vercel URL
- [ ] All pages load without errors
- [ ] Navigation works correctly
- [ ] Language switcher (English/Urdu) works
- [ ] Search functionality works
- [ ] Mermaid diagrams render
- [ ] Dark mode toggle works
- [ ] Mobile responsive design works
- [ ] Update `url` in `docusaurus.config.js` to actual Vercel domain

## Continuous Deployment Workflow

```bash
# Make changes to content
# Example: Edit a chapter
code docs/chapters/chapter-02-ros2-architecture/index.md

# Test locally
npm start

# Commit changes
git add .
git commit -m "Update Chapter 2: Add ROS2 architecture diagrams"

# Push to GitHub (triggers automatic Vercel deployment)
git push origin main

# Vercel automatically:
# 1. Detects the push
# 2. Builds the site
# 3. Deploys to production
# 4. Sends you a notification

# Visit your site in 2-3 minutes to see updates live
```

## Deployment Commands

```bash
# Deploy to preview (development)
vercel

# Deploy to production
vercel --prod

# View deployment logs
vercel logs

# List all deployments
vercel ls

# Inspect a specific deployment
vercel inspect <deployment-url>

# Remove a deployment
vercel rm <deployment-url>

# Open project in browser
vercel open
```

## Rollback to Previous Version

If something goes wrong:

1. Go to Vercel Dashboard → Deployments
2. Find the last working deployment
3. Click "..." → "Promote to Production"
4. Confirm rollback

Your site instantly reverts to the previous version.

## Cost

**Free Tier Includes:**
- Unlimited deployments
- 100 GB bandwidth per month
- Automatic HTTPS
- Global CDN
- Preview deployments
- Analytics (basic)

**Pro Tier ($20/month):**
- 1 TB bandwidth
- Advanced analytics
- Password protection
- Team collaboration

For a textbook site, the free tier is typically sufficient.

## Support Resources

- **Vercel Documentation**: https://vercel.com/docs
- **Docusaurus on Vercel**: https://vercel.com/guides/deploying-docusaurus-with-vercel
- **Vercel Support**: https://vercel.com/support
- **Community**: https://github.com/vercel/vercel/discussions

## Next Steps

1. **Deploy Now**: Follow Option 1 or Option 2 above
2. **Test Thoroughly**: Verify all features work on production
3. **Share Your Site**: Your textbook is now live and accessible worldwide
4. **Monitor**: Check analytics and performance regularly
5. **Iterate**: Continue adding content - deployments are automatic

---

**Your site is ready to deploy!**

Current build status: ✅ Build successful (both English and Urdu locales)

Deploy now at: https://vercel.com/new
