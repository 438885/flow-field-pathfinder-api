/*! coi-serviceworker v0.1.7 - Guido Zuidhof and contributors, licensed under MIT */
/*! Converted to TypeScript by jricht@pm.me */

/**
 * @fileoverview
 * Enables Cross-Origin Isolation (COOP + COEP) via a Service Worker in environments
 * where you cannot control HTTP response headers (e.g., GitHub Pages, static CDNs).
 *
 * Cross-Origin Isolation is required by modern browsers to unlock powerful APIs like:
 * - `SharedArrayBuffer` & WebAssembly threads
 * - High-resolution `performance.now()` timers
 * - `performance.measureUserAgentSpecificMemory()`
 *
 * This single file is designed to run in **two** contexts:
 * 1. **Browser Main Thread (`Window`)**: Registers the Service Worker and manages reloads.
 * 2. **Service Worker Thread (`ServiceWorkerGlobalScope`)**: Intercepts `fetch` requests
 *    and injects `Cross-Origin-Opener-Policy` and `Cross-Origin-Embedder-Policy` headers.
 */

/**
 * Reason why the page is being reloaded by `coi-serviceworker`.
 * - `"coepdegrade"`: Reloading to fall back from `credentialless` to `require-corp`.
 * - `"updatefound"`: A newer version of the Service Worker was installed and activated.
 * - `"notcontrolling"`: The Service Worker is active, but not yet controlling this page.
 */
export type CoiReloadReason = "coepdegrade" | "updatefound" | "notcontrolling";

/**
 * Configuration options for `coi-serviceworker`.
 * Can be assigned to `window.coi` before loading the script or passed to `initCoiServiceWorker()`.
 */
export interface CoiConfig {
  /**
   * Returns whether the Service Worker should be registered if the page is not yet isolated.
   * Defaults to preventing registration if the page was just reloaded by self.
   */
  shouldRegister?: () => boolean;

  /**
   * Returns whether an existing controlling Service Worker should be unregistered.
   */
  shouldDeregister?: () => boolean;

  /**
   * Returns whether `Cross-Origin-Embedder-Policy: credentialless` should be preferred
   * over `require-corp` (avoids needing `Cross-Origin-Resource-Policy` on third-party subresources).
   */
  coepCredentialless?: () => boolean;

  /**
   * Returns whether to automatically fall back ("degrade") from `credentialless` to
   * `require-corp` if the browser does not support `credentialless` (e.g., older Safari/Firefox).
   */
  coepDegrade?: () => boolean;

  /**
   * Custom reload handler called when the Service Worker needs to refresh the page
   * to apply headers or degrade COEP mode.
   */
  doReload?: (reason?: CoiReloadReason) => void;

  /**
   * When `true`, suppresses informational `console.log` and `console.error` messages.
   */
  quiet?: boolean;

  /**
   * Fallback URL to the compiled Service Worker script when loaded as an ES module
   * (where `document.currentScript` is `null`).
   * @default "/coi-serviceworker.js"
   */
  scriptUrl?: string;

  /**
   * Optional Service Worker registration scope (e.g., `"/"` or `"/subpath/"`).
   * If omitted, defaults to the directory containing the Service Worker script.
   */
  scope?: string;

  /**
   * Optional Service Worker script evaluation mode (`"classic"` or `"module"`).
   * Set to `"module"` if the compiled Service Worker file retains ES module `export` statements.
   * @default "classic"
   */
  workerType?: WorkerType;
}

/**
 * Snapshot of the current Cross-Origin Isolation and Service Worker state.
 */
export interface CoiStatus {
  /** `true` if `globalThis.crossOriginIsolated` is active. */
  crossOriginIsolated: boolean;
  /** `true` if the `SharedArrayBuffer` constructor is defined in this global scope. */
  sharedArrayBufferAvailable: boolean;
  /** `true` if the browser supports `navigator.serviceWorker`. */
  serviceWorkerSupported: boolean;
  /** `true` if a Service Worker is actively controlling the current page. */
  serviceWorkerControlling: boolean;
  /** `true` if running in a Secure Context (HTTPS or `localhost`). */
  isSecureContext: boolean;
  /** `true` if the Service Worker is configured to use `COEP: credentialless`. */
  coepCredentialless: boolean;
  /**
   * High-level summary of how isolation is currently achieved:
   * - `"isolated-headers"`: Isolated natively via server HTTP response headers.
   * - `"isolated-sw"`: Isolated via the `coi-serviceworker` fetch interceptor.
   * - `"degraded"`: Service worker is active or attempted fallback, but page is still not isolated.
   * - `"unisolated"`: Not isolated and no Service Worker is controlling the page.
   */
  mode: "isolated-headers" | "isolated-sw" | "degraded" | "unisolated";
}

declare global {
  interface Window {
    /** Optional global configuration object read by `coi-serviceworker`. */
    coi?: CoiConfig;
    /** Internal guard flag to prevent duplicate initialization on the main thread. */
    __coiInitialized?: boolean;
  }
}

/**
 * Messages sent from the main `Window` thread to the Service Worker via `postMessage`.
 */
type CoiMessage =
  | { type: "deregister" }
  | { type: "coepCredentialless"; value: boolean };

// ============================================================================
// Minimal Service Worker Types
// ============================================================================
// TypeScript's built-in `lib.dom.d.ts` and `lib.webworker.d.ts` cannot both be
// included in a single compilation unit without conflicting on `declare var self`.
// Defining these minimal structural interfaces allows this file to compile cleanly
// in standard DOM projects without requiring a separate tsconfig for the worker.

interface CoiWindowClient {
  readonly url: string;
  navigate(url: string): Promise<CoiWindowClient | null>;
}

interface CoiClients {
  claim(): Promise<void>;
  matchAll(): Promise<readonly CoiWindowClient[]>;
}

interface CoiExtendableEvent extends Event {
  waitUntil(f: Promise<unknown>): void;
}

interface CoiExtendableMessageEvent<T = unknown> extends CoiExtendableEvent {
  readonly data: T;
}

interface CoiFetchEvent extends CoiExtendableEvent {
  readonly request: Request;
  respondWith(r: Response | PromiseLike<Response>): void;
}

interface CoiServiceWorkerGlobalScope {
  readonly registration: ServiceWorkerRegistration;
  readonly clients: CoiClients;
  skipWaiting(): Promise<void>;
  addEventListener(
    type: "install" | "activate",
    listener: (event: CoiExtendableEvent) => void
  ): void;
  addEventListener(
    type: "message",
    listener: (event: CoiExtendableMessageEvent<CoiMessage | undefined>) => void
  ): void;
  addEventListener(
    type: "fetch",
    listener: (event: CoiFetchEvent) => void
  ): void;
}

// ============================================================================
// Internal State & Safe Storage Helpers
// ============================================================================

/**
 * Tracks whether `Cross-Origin-Embedder-Policy: credentialless` is active.
 * Updated via `initCoiServiceWorker()` in the Window context, and via the
 * `"message"` event listener in the Service Worker context.
 */
let coepCredentialless = false;

/**
 * Safely reads a key from `window.sessionStorage`.
 * Wrapped in `try/catch` because accessing `window.sessionStorage` in sandboxed
 * iframes (without `allow-same-origin`) or when cookies/site-data are blocked
 * throws a synchronous `SecurityError`.
 */
function safeSessionGet(key: string): string | null {
  try {
    return typeof window !== "undefined"
      ? window.sessionStorage.getItem(key)
      : null;
  } catch {
    return null;
  }
}

/**
 * Safely writes a key-value pair to `window.sessionStorage`.
 */
function safeSessionSet(key: string, value: string): void {
  try {
    if (typeof window !== "undefined") {
      window.sessionStorage.setItem(key, value);
    }
  } catch {
    // Ignore storage access errors in restricted/sandboxed contexts
  }
}

/**
 * Safely removes a key from `window.sessionStorage`.
 */
function safeSessionRemove(key: string): void {
  try {
    if (typeof window !== "undefined") {
      window.sessionStorage.removeItem(key);
    }
  } catch {
    // Ignore storage access errors in restricted/sandboxed contexts
  }
}

// ============================================================================
// Public API
// ============================================================================

/**
 * Inspects the current runtime environment and returns a detailed status object
 * describing Cross-Origin Isolation, `SharedArrayBuffer` availability, and
 * Service Worker state.
 *
 * Safe to call in Browser, Web Worker, Service Worker, or Node.js/SSR contexts.
 */
export function getCoiStatus(): CoiStatus {
  const isBrowser = typeof window !== "undefined";
  const crossOriginIsolated =
    typeof globalThis.crossOriginIsolated === "boolean"
      ? globalThis.crossOriginIsolated
      : false;
  const sharedArrayBufferAvailable = typeof SharedArrayBuffer !== "undefined";
  const serviceWorkerSupported =
    isBrowser &&
    typeof navigator !== "undefined" &&
    "serviceWorker" in navigator;
  const serviceWorkerControlling = Boolean(
    serviceWorkerSupported && navigator.serviceWorker?.controller
  );
  const isSecureContext =
    typeof globalThis.isSecureContext === "boolean"
      ? globalThis.isSecureContext
      : true;
  const coepHasFailed = Boolean(safeSessionGet("coiCoepHasFailed"));

  let mode: CoiStatus["mode"] = "unisolated";
  if (crossOriginIsolated) {
    // If isolated without a controlling Service Worker, the server sent native headers
    mode = serviceWorkerControlling ? "isolated-sw" : "isolated-headers";
  } else if (coepHasFailed || serviceWorkerControlling) {
    // A Service Worker is controlling (or failed previously), yet isolation is still false
    mode = "degraded";
  }

  return {
    crossOriginIsolated,
    sharedArrayBufferAvailable,
    serviceWorkerSupported,
    serviceWorkerControlling,
    isSecureContext,
    coepCredentialless,
    mode,
  };
}

/**
 * Allocates a `SharedArrayBuffer` of the specified byte length when Cross-Origin
 * Isolation (or a non-browser runtime like Node.js) permits it, gracefully falling
 * back to a standard `ArrayBuffer` when unavailable.
 *
 * @param byteLength - Size of the buffer in bytes.
 * @returns A `SharedArrayBuffer` if available and permitted, otherwise an `ArrayBuffer`.
 */
export function createSharedOrArrayBuffer(byteLength: number): ArrayBufferLike {
  if (typeof SharedArrayBuffer !== "undefined") {
    // In browsers and workers, `crossOriginIsolated` is a boolean.
    // In Node.js, `crossOriginIsolated` is undefined, and SAB is always allowed.
    const canUseSab =
      typeof globalThis.crossOriginIsolated === "boolean"
        ? globalThis.crossOriginIsolated
        : true;
    if (canUseSab) {
      try {
        return new SharedArrayBuffer(byteLength);
      } catch {
        // Fallback to standard ArrayBuffer if instantiation is restricted by browser policy
      }
    }
  }
  return new ArrayBuffer(byteLength);
}

/**
 * Allocates a `Float32Array` backed by a `SharedArrayBuffer` (if isolated)
 * or a standard `ArrayBuffer` (if unisolated).
 *
 * @param length - Number of 32-bit float elements.
 */
export function allocFloat32(length: number): Float32Array {
  return new Float32Array(
    createSharedOrArrayBuffer(length * Float32Array.BYTES_PER_ELEMENT)
  );
}

/**
 * Allocates an `Int32Array` backed by a `SharedArrayBuffer` (if isolated)
 * or a standard `ArrayBuffer` (if unisolated).
 *
 * Note: `Atomics.wait()` and `Atomics.notify()` require an `Int32Array` (or `BigInt64Array`)
 * backed by a true `SharedArrayBuffer`.
 *
 * @param length - Number of 32-bit integer elements.
 */
export function allocInt32(length: number): Int32Array {
  return new Int32Array(
    createSharedOrArrayBuffer(length * Int32Array.BYTES_PER_ELEMENT)
  );
}

/**
 * Instructs the active `coi-serviceworker` to unregister itself and reload
 * all controlled client tabs.
 */
export function deregisterCoiServiceWorker(): void {
  if (typeof window === "undefined" || !navigator.serviceWorker?.controller) {
    return;
  }
  navigator.serviceWorker.controller.postMessage({
    type: "deregister",
  } satisfies CoiMessage);
}

/**
 * Initializes the Cross-Origin Isolation Service Worker from the main browser thread.
 *
 * This function is idempotent (`window.__coiInitialized` prevents duplicate runs)
 * and handles:
 * 1. Detecting whether a controlling Service Worker failed to isolate the page
 *    (e.g., because the browser doesn't support `COEP: credentialless`) and triggering
 *    a reload to degrade to `COEP: require-corp`.
 * 2. Registering the Service Worker if `window.crossOriginIsolated === false`.
 * 3. Reloading the page once the Service Worker activates and claims control.
 *
 * @param customConfig - Optional configuration overrides merged into `window.coi`.
 * @returns The current `CoiStatus` snapshot.
 */
export function initCoiServiceWorker(customConfig?: CoiConfig): CoiStatus {
  // No-op if called outside the main browser Window context (e.g., SSR or Worker)
  if (typeof window === "undefined") {
    return getCoiStatus();
  }

  if (customConfig) {
    window.coi = { ...window.coi, ...customConfig };
  }

  // Ensure initialization logic only runs once per page load
  if (window.__coiInitialized) {
    return getCoiStatus();
  }
  window.__coiInitialized = true;

  // Check if this page load was triggered by our own `doReload()` call,
  // then immediately clear the flag so manual user refreshes aren't blocked.
  const reloadedBySelf = safeSessionGet("coiReloadedBySelf");
  safeSessionRemove("coiReloadedBySelf");
  const coepDegrading = reloadedBySelf === "coepdegrade";

  // Merge user configuration with sensible defaults
  const coi: Required<Omit<CoiConfig, "scope" | "workerType">> &
    Pick<CoiConfig, "scope" | "workerType"> = {
    shouldRegister: () => !reloadedBySelf,
    shouldDeregister: () => false,
    coepCredentialless: () => true,
    coepDegrade: () => true,
    doReload: () => window.location.reload(),
    quiet: false,
    scriptUrl: "/coi-serviceworker.js",
    ...window.coi,
  };

  const nav = navigator;
  const controlling = nav.serviceWorker && nav.serviceWorker.controller;

  // If a Service Worker is already controlling the page, but `window.crossOriginIsolated`
  // is still false, record that the initial COEP strategy (`credentialless`) failed.
  const coepHasFailed = safeSessionGet("coiCoepHasFailed");
  if (controlling && !window.crossOriginIsolated) {
    safeSessionSet("coiCoepHasFailed", "true");
  }

  // Determine if we need to reload right now to switch from `credentialless` to `require-corp`
  const reloadToDegrade =
    coi.coepDegrade() &&
    !(coepDegrading || coepHasFailed || window.crossOriginIsolated);

  // Use `credentialless` only if we aren't degrading and haven't previously failed
  coepCredentialless =
    !(reloadToDegrade || (coepHasFailed && coi.coepDegrade())) &&
    coi.coepCredentialless();

  if (controlling) {
    // Sync the chosen COEP mode (`credentialless` vs `require-corp`) to the Service Worker
    controlling.postMessage({
      type: "coepCredentialless",
      value: coepCredentialless,
    } satisfies CoiMessage);

    if (reloadToDegrade) {
      if (!coi.quiet) {
        console.log("Reloading page to degrade COEP.");
      }
      safeSessionSet("coiReloadedBySelf", "coepdegrade");
      coi.doReload("coepdegrade");
    }

    if (coi.shouldDeregister()) {
      controlling.postMessage({ type: "deregister" } satisfies CoiMessage);
    }
  }

  // If the page is not yet Cross-Origin Isolated, register the Service Worker
  if (window.crossOriginIsolated === false && coi.shouldRegister()) {
    // Service Workers and Cross-Origin Isolation both require HTTPS (or localhost)
    if (!window.isSecureContext) {
      if (!coi.quiet) {
        console.log(
          "COOP/COEP Service Worker not registered, a secure context is required."
        );
      }
      return getCoiStatus();
    }

    // Firefox Private Browsing and some WebViews disable `navigator.serviceWorker`
    if (!nav.serviceWorker) {
      if (!coi.quiet) {
        console.error(
          "COOP/COEP Service Worker not registered, perhaps due to private mode."
        );
      }
      return getCoiStatus();
    }

    // Prefer the `<script src="...">` URL if loaded via a classic script tag;
    // otherwise fall back to `coi.scriptUrl` (for ES module bundlers).
    const currentScript = window.document
      .currentScript as HTMLScriptElement | null;
    const scriptSrc = currentScript?.src || coi.scriptUrl;

    if (!scriptSrc) {
      if (!coi.quiet) {
        console.error(
          "COOP/COEP Service Worker not registered, scriptUrl is missing."
        );
      }
      return getCoiStatus();
    }

    const registerOptions: RegistrationOptions | undefined =
      coi.scope || coi.workerType
        ? {
            ...(coi.scope ? { scope: coi.scope } : {}),
            ...(coi.workerType ? { type: coi.workerType } : {}),
          }
        : undefined;

    nav.serviceWorker.register(scriptSrc, registerOptions).then(
      (registration) => {
        if (!coi.quiet) {
          console.log(
            "COOP/COEP Service Worker registered",
            registration.scope
          );
        }

        // Listen for a new/updated Service Worker installing in the background
        // and wait until it reaches `"activated"` before reloading the page.
        registration.addEventListener("updatefound", () => {
          const installing = registration.installing;
          if (!installing) return;

          installing.addEventListener("statechange", () => {
            if (installing.state === "activated") {
              if (!coi.quiet) {
                console.log(
                  "Reloading page to make use of updated COOP/COEP Service Worker."
                );
              }
              safeSessionSet("coiReloadedBySelf", "updatefound");
              coi.doReload("updatefound");
            }
          });
        });

        // If the Service Worker is already active (e.g., from a previous tab),
        // but isn't controlling this document yet, reload immediately to claim control.
        if (registration.active && !nav.serviceWorker.controller) {
          if (!coi.quiet) {
            console.log(
              "Reloading page to make use of COOP/COEP Service Worker."
            );
          }
          safeSessionSet("coiReloadedBySelf", "notcontrolling");
          coi.doReload("notcontrolling");
        }
      },
      (err) => {
        if (!coi.quiet) {
          console.error(
            "COOP/COEP Service Worker failed to register:",
            err
          );
        }
      }
    );
  }

  return getCoiStatus();
}

// ============================================================================
// Runtime Context Detection & Execution
// ============================================================================

if (
  typeof window === "undefined" &&
  typeof self !== "undefined" &&
  "skipWaiting" in self
) {
  // --------------------------------------------------------------------------
  // 1. Service Worker Context
  // --------------------------------------------------------------------------
  const sw = self as unknown as CoiServiceWorkerGlobalScope;

  // Activate immediately upon installation without waiting for older tabs to close
  sw.addEventListener("install", (event) => {
    event.waitUntil(sw.skipWaiting());
  });

  // Immediately claim control over all open clients within scope
  sw.addEventListener("activate", (event) => {
    event.waitUntil(sw.clients.claim());
  });

  // Handle control messages from the main Window thread
  sw.addEventListener("message", (event) => {
    if (!event.data) return;

    if (event.data.type === "deregister") {
      event.waitUntil(
        sw.registration
          .unregister()
          .then(() => sw.clients.matchAll())
          .then((clients) =>
            // Navigate each client to its own URL to force a fresh non-SW page load
            Promise.all(clients.map((client) => client.navigate(client.url)))
          )
      );
    } else if (event.data.type === "coepCredentialless") {
      coepCredentialless = event.data.value;
    }
  });

  // Intercept all network requests and inject COOP/COEP/CORP response headers
  sw.addEventListener("fetch", (event) => {
    const request = event.request;

    // Workaround for a known Chromium bug: opening DevTools triggers a fetch with
    // `cache: "only-if-cached"` and `mode: "no-cors"`, which throws if passed to `fetch()`.
    if (request.cache === "only-if-cached" && request.mode !== "same-origin") {
      return;
    }

    // When using `COEP: credentialless`, strip credentials from `no-cors` requests
    const modifiedRequest =
      coepCredentialless && request.mode === "no-cors"
        ? new Request(request, { credentials: "omit" })
        : request;

    event.respondWith(
      fetch(modifiedRequest)
        .then((response) => {
          // Opaque responses (status === 0, e.g. cross-origin `no-cors`) have immutable
          // headers and body; return them untouched.
          if (response.status === 0) {
            return response;
          }

          // Clone the existing headers and add the required Cross-Origin Isolation policies
          const newHeaders = new Headers(response.headers);
          newHeaders.set(
            "Cross-Origin-Embedder-Policy",
            coepCredentialless ? "credentialless" : "require-corp"
          );
          if (!coepCredentialless) {
            newHeaders.set("Cross-Origin-Resource-Policy", "cross-origin");
          }
          newHeaders.set("Cross-Origin-Opener-Policy", "same-origin");

          return new Response(response.body, {
            status: response.status,
            statusText: response.statusText,
            headers: newHeaders,
          });
        })
        .catch((err) => {
          console.error(err);
          return Response.error();
        })
    );
  });
} else if (
  typeof window !== "undefined" &&
  (window.document?.currentScript !== null || window.coi !== undefined)
) {
  // --------------------------------------------------------------------------
  // 2. Browser Window Context (Auto-Initialization)
  // --------------------------------------------------------------------------
  // Only auto-initialize when loaded via a classic `<script src="...">` tag
  // (`document.currentScript !== null`) or when `window.coi` was explicitly
  // configured prior to importing this module.
  // When imported as an ES module (`import { initCoiServiceWorker } from ...`),
  // this allows the caller to invoke `initCoiServiceWorker(customConfig)` manually.
  initCoiServiceWorker();
}