import { readFile } from "node:fs/promises";

export async function loadWorkerModule(path) {
  const source = await readFile(path, "utf8");
  const testableSource = source.replace(
    'import { DurableObject } from "cloudflare:workers";',
    `class DurableObject {
      constructor(ctx, env) {
        this.ctx = ctx;
        this.env = env;
      }
    }`,
  );
  if (testableSource === source) {
    throw new Error("Cloudflare DurableObject import was not found");
  }
  const moduleUrl = `data:text/javascript;base64,${Buffer.from(testableSource).toString("base64")}`;
  return import(moduleUrl);
}

export function makeCounterEnvironment(overrides = {}) {
  const calls = [];
  const counter = {
    count: async () => 7,
    report: async () => ({ count: 8, recorded: true }),
    ...overrides,
  };
  return {
    calls,
    env: {
      UPDATE_COUNTER: {
        getByName(name, options) {
          calls.push({ name, options });
          return counter;
        },
      },
    },
  };
}

export async function responseJson(response) {
  return { status: response.status, headers: response.headers, body: await response.json() };
}
