"use strict";

let current;
let timer;

async function renew(session) {
  try {
    const response = await fetch("/api/heartbeat", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ session_id: session.id }),
    });
    if (response.status === 409 && current === session) {
      current = undefined;
      self.postMessage({ lost: session.id });
    }
  } catch {
    // Temporary network failures are handled by the existing arbiter timeout.
  } finally {
    if (current === session) timer = setTimeout(() => renew(session), session.period);
  }
}

self.addEventListener("message", ({ data: session }) => {
  clearTimeout(timer);
  current = session;
  if (current) renew(current);
});
