import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import useBaseUrl from '@docusaurus/useBaseUrl';
import Translate, {translate} from '@docusaurus/Translate';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>

        <div className={styles.buttons}>
          <Link className="button button--secondary button--lg" to="/docs/intro">
            <Translate id="homepage.hero.button">Start Learning →</Translate>
          </Link>
        </div>
      </div>

    </header>
  );
}

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>
                <Translate id="homepage.features.interactive.title">
                  Interactive Learning
                </Translate>
              </h3>
              <p>
                <Translate id="homepage.features.interactive.description">
                  Hands-on labs and exercises to master Physical AI and Humanoid Robotics concepts.
                </Translate>
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>
                <Translate id="homepage.features.comprehensive.title">
                  Comprehensive Content
                </Translate>
              </h3>
              <p>
                <Translate id="homepage.features.comprehensive.description">
                  Learn robot manipulation, locomotion, perception, and control with detailed tutorials.
                </Translate>
              </p>
            </div>
          </div>
          <div className="col col--4">
            <div className="text--center padding-horiz--md">
              <h3>
                <Translate id="homepage.features.multilingual.title">
                  Multilingual Support
                </Translate>
              </h3>
              <p>
                <Translate id="homepage.features.multilingual.description">
                  Available in English and Urdu for broader accessibility.
                </Translate>
              </p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title={translate({
        id: 'homepage.layout.title',
        message: `Welcome to ${siteConfig.title}`,
      })}
      description={translate({
        id: 'homepage.layout.description',
        message: 'Learn Physical AI and Humanoid Robotics with interactive tutorials and hands-on labs',
      })}
    >
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
