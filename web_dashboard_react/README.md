# ESP32 Air Mouse - React Dashboard

This is a modern React application to visualize data from your ESP32 Air Mouse via Firebase.

## Features
- Real-time charts for Pitch/Roll and Gyroscope data
- Battery monitoring
- Connection history log
- ML Gesture event log
- Responsive design with Tailwind CSS

## Setup & Run

1.  **Install Dependencies**:
    Open a terminal in this folder (`web_dashboard_react`) and run:
    ```bash
    npm install
    ```

2.  **Run Locally**:
    ```bash
    npm run dev
    ```
    Open the link shown (usually `http://localhost:5173`).

## Deployment (Upload to Web)

To upload this to the public web using Firebase Hosting:

1.  **Install Firebase Tools** (if not already installed):
    ```bash
    npm install -g firebase-tools
    ```

2.  **Login**:
    ```bash
    firebase login
    ```

3.  **Initialize**:
    ```bash
    firebase init hosting
    ```
    - Select your project (`air-mouse-a0f08`).
    - Public directory: `dist`
    - Configure as single-page app: **Yes**
    - Automatic builds with GitHub: **No** (unless you want that)

4.  **Build & Deploy**:
    ```bash
    npm run build
    firebase deploy
    ```

Your site will be live at `https://air-mouse-a0f08.web.app`.
