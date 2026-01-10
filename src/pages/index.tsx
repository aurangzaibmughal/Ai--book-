import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';
import styles from './index.module.css';

type Feature = {
  title: string;
  description: string;
};

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>

        <div className={styles.buttons}>
          <Link className="button button--secondary button--lg" to="/docs/intro">
            Start Learning →
          </Link>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  const features: Feature[] = [
    {
      title: 'Interactive Learning',
      description: 'Hands-on labs and exercises to master Physical AI and Humanoid Robotics concepts.',
    },
    {
      title: 'Comprehensive Content',
      description: 'Learn robot manipulation, locomotion, perception, and control with detailed tutorials.',
    },
    {
      title: 'Multilingual Support',
      description: 'Available in English and Urdu for broader accessibility.',
    },
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {features.map((feature, idx) => (
            <div key={idx} className="col col--4">
              <div className="text--center padding-horiz--md">
                <h3>{feature.title}</h3>
                <p>{feature.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Learn Physical AI and Humanoid Robotics with interactive tutorials and hands-on labs"
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
