import React, {type ReactNode, useEffect, useState} from 'react';
import DocItem from '@theme-original/DocItem';
import type DocItemType from '@theme/DocItem';
import type {WrapperProps} from '@docusaurus/types';
import TranslationButton from '@site/src/components/TranslationButton';

type Props = WrapperProps<typeof DocItemType>;

export default function DocItemWrapper(props: Props): ReactNode {
  const [pageContent, setPageContent] = useState<string>('');

  useEffect(() => {
    // After the doc item renders, grab the visible markdown text from the page
    const grabRenderedMarkdown = () => {
      // Common selector used by Docusaurus theme for rendered markdown
      const el = document.querySelector('.markdown, .theme-doc-markdown');
      if (el) {
        setPageContent((el as HTMLElement).innerText || '');
      }
    };

    // Slight delay to let the DocItem render
    const t = setTimeout(grabRenderedMarkdown, 150);
    return () => clearTimeout(t);
  }, [props.location.pathname]);

  return (
    <>
      <DocItem {...props} />
      {pageContent && <TranslationButton textToTranslate={pageContent} />}
    </>
  );
}
