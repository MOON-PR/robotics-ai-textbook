import type {ReactNode} from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: '🚀 Comprehensive Curriculum',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Master Physical AI and Humanoid Robotics through 10 carefully structured chapters,
        from fundamentals to advanced integration of AI systems with robotics platforms.
      </>
    ),
  },
  {
    title: '🤖 Hands-On Projects',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Work with real frameworks like ROS 2, Gazebo, Isaac Sim, and NVIDIA robotics tools.
        Build practical applications that demonstrate real-world robotics solutions.
      </>
    ),
  },
  {
    title: '🧠 AI-Powered Assistant',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        Interactive RAG chatbot answers your questions instantly with citations from the textbook.
        Personalized learning paths adapt to your skill level (Beginner, Intermediate, Advanced).
      </>
    ),
  },
];

function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
