/*! coi-serviceworker v0.1.7 - Guido Zuidhof and contributors, licensed under MIT */
/*! Converted to TypeScript by jricht@pm.me */

export type CoiReloadReason = "coepdegrade" | "updatefound" | "notcontrolling";

export interface CoiConfig {
  shouldRegister?: () => boolean;
  shouldDeregister?: () => boolean;
  coepCredentialless?: () => boolean;
  coepDegrade?: () => boolean;
  doReload?: (reason?: CoiReloadReason) => void;
  quiet?: boolean;
}

declare global {
  interface Window {
    coi?: CoiConfig;
  }
}

type CoiMessage =
  | { type: "deregister" }
  | { type: "coepCredentialless"; value: boolean };

// Minimal Service Worker types to avoid conflicts between lib.dom.d.ts and lib.webworker.d.ts
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

let coepCredentialless = false;

if (typeof window === "undefined") {
  // --- Service Worker Context ---
  const sw = self as unknown as CoiServiceWorkerGlobalScope;

  sw.addEventListener("install", () => {
    sw.skipWaiting();
  });

  sw.addEventListener("activate", (event) => {
    event.waitUntil(sw.clients.claim());
  });

  sw.addEventListener("message", (event) => {
    if (!event.data) return;

    if (event.data.type === "deregister") {
      sw.registration
        .unregister()
        .then(() => sw.clients.matchAll())
        .then((clients) => {
          clients.forEach((client) => client.navigate(client.url));
        });
    } else if (event.data.type === "coepCredentialless") {
      coepCredentialless = event.data.value;
    }
  });

  sw.addEventListener("fetch", (event) => {
    const request = event.request;

    if (request.cache === "only-if-cached" && request.mode !== "same-origin") {
      return;
    }

    const modifiedRequest =
      coepCredentialless && request.mode === "no-cors"
        ? new Request(request, { credentials: "omit" })
        : request;

    event.respondWith(
      fetch(modifiedRequest)
        .then((response) => {
          if (response.status === 0) {
            return response;
          }

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
} else {
  // --- Browser Window Context ---
  (() => {
    const reloadedBySelf = window.sessionStorage.getItem("coiReloadedBySelf");
    window.sessionStorage.removeItem("coiReloadedBySelf");
    const coepDegrading = reloadedBySelf === "coepdegrade";

    const coi: Required<CoiConfig> = {
      shouldRegister: () => !reloadedBySelf,
      shouldDeregister: () => false,
      coepCredentialless: () => true,
      coepDegrade: () => true,
      doReload: () => window.location.reload(),
      quiet: false,
      ...window.coi,
    };

    const nav = navigator;
    const controlling = nav.serviceWorker && nav.serviceWorker.controller;

    if (controlling && !window.crossOriginIsolated) {
      window.sessionStorage.setItem("coiCoepHasFailed", "true");
    }
    const coepHasFailed = window.sessionStorage.getItem("coiCoepHasFailed");

    if (controlling) {
      const reloadToDegrade =
        coi.coepDegrade() && !(coepDegrading || window.crossOriginIsolated);

      controlling.postMessage({
        type: "coepCredentialless",
        value:
          !(reloadToDegrade || (coepHasFailed && coi.coepDegrade())) &&
          coi.coepCredentialless(),
      } satisfies CoiMessage);

      if (reloadToDegrade) {
        if (!coi.quiet) {
          console.log("Reloading page to degrade COEP.");
        }
        window.sessionStorage.setItem("coiReloadedBySelf", "coepdegrade");
        coi.doReload("coepdegrade");
      }

      if (coi.shouldDeregister()) {
        controlling.postMessage({ type: "deregister" } satisfies CoiMessage);
      }
    }

    if (window.crossOriginIsolated === false && coi.shouldRegister()) {
      if (!window.isSecureContext) {
        if (!coi.quiet) {
          console.log(
            "COOP/COEP Service Worker not registered, a secure context is required."
          );
        }
        return;
      }

      if (!nav.serviceWorker) {
        if (!coi.quiet) {
          console.error(
            "COOP/COEP Service Worker not registered, perhaps due to private mode."
          );
        }
        return;
      }

      const currentScript = window.document
        .currentScript as HTMLScriptElement | null;

      if (!currentScript?.src) {
        if (!coi.quiet) {
          console.error(
            "COOP/COEP Service Worker not registered, currentScript.src is missing."
          );
        }
        return;
      }

      nav.serviceWorker.register(currentScript.src).then(
        (registration) => {
          if (!coi.quiet) {
            console.log(
              "COOP/COEP Service Worker registered",
              registration.scope
            );
          }

          registration.addEventListener("updatefound", () => {
            if (!coi.quiet) {
              console.log(
                "Reloading page to make use of updated COOP/COEP Service Worker."
              );
            }
            window.sessionStorage.setItem("coiReloadedBySelf", "updatefound");
            coi.doReload("updatefound");
          });

          if (registration.active && !nav.serviceWorker.controller) {
            if (!coi.quiet) {
              console.log(
                "Reloading page to make use of COOP/COEP Service Worker."
              );
            }
            window.sessionStorage.setItem(
              "coiReloadedBySelf",
              "notcontrolling"
            );
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
  })();
}