import React, {type ReactNode} from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type {WrapperProps} from '@docusaurus/types';
import Chatbot from '@site/src/components/Chatbot';
import { AuthProvider } from '@site/src/contexts/AuthContext'; // Import AuthProvider

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): ReactNode {
  return (
    <AuthProvider> {/* Wrap the entire layout with AuthProvider */}
      <Layout {...props} />
      <Chatbot />
    </AuthProvider>
  );
}
