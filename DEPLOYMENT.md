# Deployment Guide

## Current Deployment Status

### Production URLs
- **Primary**: https://ai-book-five-mauve.vercel.app
- **Latest**: https://ai-book-eywehd658-aurangzaib-mughals-projects.vercel.app
- **Vercel Dashboard**: https://vercel.com/aurangzaib-mughals-projects/ai-book

### Deployment Configuration
- **Platform**: Vercel
- **Framework**: Docusaurus
- **Build Command**: `npm run build`
- **Output Directory**: `build`
- **Node Version**: >=18.0

## Automatic Deployments

### GitHub Integration
The project is connected to GitHub repository: `aurangzaibmughal/Ai--book-`

**Automatic deployment triggers:**
- **Production**: Pushes to `main` branch
- **Preview**: Pull requests and other branches

### Manual Deployment
```bash
# Deploy to preview
vercel

# Deploy to production
vercel --prod
```

## Custom Domain Setup

### Prerequisites
- Own a domain name
- Access to domain DNS settings

### Steps to Add Custom Domain

1. **Via Vercel Dashboard** (Recommended):
   - Go to: https://vercel.com/aurangzaib-mughals-projects/ai-book/settings/domains
   - Click "Add Domain"
   - Enter your domain (e.g., `ai-book.yourdomain.com` or `yourdomain.com`)
   - Follow DNS configuration instructions

2. **Via Vercel CLI**:
   ```bash
   vercel domains add yourdomain.com
   ```

3. **DNS Configuration**:
   Add one of these records to your DNS provider:

   **For root domain (yourdomain.com):**
   ```
   Type: A
   Name: @
   Value: 76.76.21.21
   ```

   **For subdomain (ai-book.yourdomain.com):**
   ```
   Type: CNAME
   Name: ai-book
   Value: cname.vercel-dns.com
   ```

4. **Update Docusaurus Config**:
   After adding custom domain, update `docusaurus.config.js`:
   ```javascript
   url: 'https://yourdomain.com',
   ```

5. **Redeploy**:
   ```bash
   vercel --prod
   ```

### SSL/HTTPS
Vercel automatically provisions SSL certificates for all domains. HTTPS is enabled by default.

## Environment Variables

### Current Status
No environment variables are currently configured (not needed for static Docusaurus site).

### When You Might Need Environment Variables

**Common use cases:**
- API keys for external services
- Analytics tracking IDs (Google Analytics, etc.)
- Feature flags
- Build-time configuration

### Adding Environment Variables

1. **Via Vercel Dashboard**:
   - Go to: https://vercel.com/aurangzaib-mughals-projects/ai-book/settings/environment-variables
   - Click "Add New"
   - Set name, value, and environments (Production/Preview/Development)

2. **Via Vercel CLI**:
   ```bash
   # Add environment variable
   vercel env add VARIABLE_NAME

   # List environment variables
   vercel env ls

   # Remove environment variable
   vercel env rm VARIABLE_NAME
   ```

3. **Using in Docusaurus**:
   Environment variables must be prefixed with `DOCUSAURUS_` to be available in the browser:
   ```javascript
   // In your code
   const apiKey = process.env.DOCUSAURUS_API_KEY;
   ```

### Local Development with Environment Variables
Create a `.env` file (already in .gitignore):
```bash
DOCUSAURUS_API_KEY=your_key_here
```

## Build Configuration

### Current Build Settings
- **Framework Preset**: Docusaurus
- **Build Command**: `npm run build`
- **Output Directory**: `build`
- **Install Command**: `npm install`

### Customizing Build Settings
Edit via Vercel Dashboard:
https://vercel.com/aurangzaib-mughals-projects/ai-book/settings

## Monitoring and Logs

### Deployment Logs
View deployment logs in Vercel Dashboard:
https://vercel.com/aurangzaib-mughals-projects/ai-book

### Analytics
Vercel provides built-in analytics:
https://vercel.com/aurangzaib-mughals-projects/ai-book/analytics

## Troubleshooting

### Build Failures
1. Check build logs in Vercel Dashboard
2. Test build locally: `npm run build`
3. Verify Node version matches requirements (>=18.0)

### Domain Issues
1. Verify DNS records are correctly configured
2. Wait for DNS propagation (can take up to 48 hours)
3. Check domain status in Vercel Dashboard

### Git Push Issues
If you encounter SSH permission errors:
```bash
# Check SSH key
ssh -T git@github.com

# Or use HTTPS instead
git remote set-url origin https://github.com/aurangzaibmughal/Ai--book-.git
```

## Rollback

### To Previous Deployment
1. Go to: https://vercel.com/aurangzaib-mughals-projects/ai-book
2. Find the deployment you want to rollback to
3. Click "..." menu → "Promote to Production"

### Via CLI
```bash
vercel rollback
```

## Performance Optimization

### Vercel Edge Network
Your site is automatically distributed across Vercel's global Edge Network for optimal performance.

### Caching
Static assets are automatically cached with optimal cache headers.

### Image Optimization
Consider using Vercel's Image Optimization for better performance:
https://vercel.com/docs/image-optimization

## Security

### HTTPS
- Automatically enabled for all domains
- SSL certificates auto-renewed

### Headers
Configure security headers in `vercel.json` if needed:
```json
{
  "headers": [
    {
      "source": "/(.*)",
      "headers": [
        {
          "key": "X-Content-Type-Options",
          "value": "nosniff"
        },
        {
          "key": "X-Frame-Options",
          "value": "DENY"
        }
      ]
    }
  ]
}
```

## Additional Resources

- [Vercel Documentation](https://vercel.com/docs)
- [Docusaurus Deployment Guide](https://docusaurus.io/docs/deployment)
- [Vercel CLI Reference](https://vercel.com/docs/cli)
