import { initializeApp } from "firebase/app";
import { getDatabase } from "firebase/database";

const firebaseConfig = {
  apiKey: "AIzaSyC9yJ8ztQfsYgIlr3ttqWFaagpQrrQ8es4",
  databaseURL: "https://air-mouse-a0f08-default-rtdb.firebaseio.com/"
};

const app = initializeApp(firebaseConfig);
export const db = getDatabase(app);
