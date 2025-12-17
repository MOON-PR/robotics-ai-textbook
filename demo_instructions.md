# Demo Instructions: AI-Native Textbook + RAG Chatbot

This document provides instructions on how to set up and run the AI-Native Textbook and RAG Chatbot system locally.

## Prerequisites

- Docker Desktop (for Qdrant and FastAPI backend)
- Node.js (v18 or higher) and npm/yarn (for Docusaurus frontend)
- Python (v3.11 or higher) and pip (for backend development if not using Docker build)

## Setup Steps

### 1. Clone the Repository

If you haven't already, clone this repository:

```bash
git clone <your-repository-url>
cd Robotics-Course-Book
```

### 2. Prepare the Backend & Qdrant (Docker Compose)

Navigate to the root of the project and start the Docker services:

```bash
docker compose up -d
```

This will:
- Pull the `qdrant/qdrant:latest` image and start a Qdrant vector database container.
- Build the FastAPI backend Docker image (based on `backend/Dockerfile`) and start a container for it.

Wait a few moments for the services to fully start. You can check their status with `docker compose ps`.

**Important**: The first time the backend starts, it will initialize the RAG pipeline, which involves loading markdown content, generating embeddings, and indexing them in Qdrant. This might take a few minutes depending on your system's performance.

### 3. Prepare and Run the Docusaurus Frontend

Navigate to the `frontend` directory:

```bash
cd frontend
```

Install the Node.js dependencies:

```bash
npm install # or yarn install
```

Start the Docusaurus development server:

```bash
npm start # or yarn start
```

This will open your browser to `http://localhost:3000` (or another available port).

### 4. Interact with the Application

- **AI-Native Textbook**: Browse the chapters in the left sidebar.
- **RAG Chatbot**: A floating chat button should appear on the bottom-right of the page. Click it to open the chat window.
  - Type your questions related to the textbook content.
  - The chatbot will provide answers grounded in the textbook and cite the source chapters.
- **Personalization**:
  - Navigate to `/profile` (or click on a profile link if added to navbar/footer).
  - Log in with any username and select a learning level (Beginner, Intermediate, Advanced).
  - The chatbot's responses should adapt based on your selected level.
- **Urdu Translation**: On any chapter page, you should see a "Translate to Urdu" button. Click it to see a placeholder Urdu translation of the page content.

## Stopping the Application

To stop the backend and Qdrant containers:

```bash
docker compose down
```

To stop the Docusaurus frontend, press `Ctrl+C` in the terminal where `npm start` is running.

## Troubleshooting

- If the backend or Qdrant fails to start, check Docker logs (`docker compose logs`).
- If the frontend fails, ensure all `npm install` dependencies are met and check browser console for errors.
- Ensure your system has sufficient disk space, especially for Docker images and Python dependencies.
