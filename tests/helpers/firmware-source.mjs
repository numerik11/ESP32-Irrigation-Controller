import { readFile } from "node:fs/promises";

const FUNCTION_PREFIX = /(?:^|\n)\s*[^;{}\n]*\b/;

export async function readFirmware(path) {
  return readFile(path, "utf8");
}

/**
 * Return a complete function definition from an Arduino sketch.
 *
 * The scanner understands comments and string/character literals so braces in
 * diagnostics do not end the definition early.  Tests compile the returned
 * definition, which keeps the exercised logic tied to the real .ino source.
 */
export function extractFunction(source, name) {
  const escapedName = name.replace(/[.*+?^${}()|[\]\\]/g, "\\$&");
  const matcher = new RegExp(
    `${FUNCTION_PREFIX.source}${escapedName}\\s*\\(([^)]*)\\)\\s*\\{`,
    "m",
  );
  const match = matcher.exec(source);
  if (!match) throw new Error(`Could not find function ${name}`);

  const definitionStart = match.index + (match[0].startsWith("\n") ? 1 : 0);
  const openBrace = source.indexOf("{", match.index + match[0].lastIndexOf(name));
  let depth = 0;
  let state = "code";

  for (let i = openBrace; i < source.length; i += 1) {
    const char = source[i];
    const next = source[i + 1];

    if (state === "line-comment") {
      if (char === "\n") state = "code";
      continue;
    }
    if (state === "block-comment") {
      if (char === "*" && next === "/") {
        state = "code";
        i += 1;
      }
      continue;
    }
    if (state === "string" || state === "character") {
      if (char === "\\") {
        i += 1;
        continue;
      }
      if ((state === "string" && char === '"') || (state === "character" && char === "'")) {
        state = "code";
      }
      continue;
    }

    if (char === "/" && next === "/") {
      state = "line-comment";
      i += 1;
    } else if (char === "/" && next === "*") {
      state = "block-comment";
      i += 1;
    } else if (char === '"') {
      state = "string";
    } else if (char === "'") {
      state = "character";
    } else if (char === "{") {
      depth += 1;
    } else if (char === "}") {
      depth -= 1;
      if (depth === 0) return source.slice(definitionStart, i + 1).trim();
    }
  }

  throw new Error(`Function ${name} has an unmatched brace`);
}

function parameterNames(definition, name) {
  const signature = new RegExp(`\\b${name}\\s*\\(([^)]*)\\)`).exec(definition);
  if (!signature) throw new Error(`Could not parse parameters for ${name}`);
  if (!signature[1].trim()) return [];
  return signature[1].split(",").map((parameter) => {
    const withoutDefault = parameter.split("=")[0].trim();
    const identifier = /([A-Za-z_]\w*)\s*$/.exec(withoutDefault);
    if (!identifier) throw new Error(`Could not parse parameter: ${parameter}`);
    return identifier[1];
  });
}

function replaceSignature(definition, name) {
  const parameters = parameterNames(definition, name);
  const signature = new RegExp(`^[^{]*\\b${name}\\s*\\([^)]*\\)`);
  return definition.replace(signature, `function ${name}(${parameters.join(", ")})`);
}

/**
 * Translate the small, scalar-only subset of C++ used by the tested helpers to
 * JavaScript and evaluate it.  This is intentionally not a general C++ parser;
 * unsupported constructs fail while compiling the test rather than silently
 * falling back to a second implementation.
 */
export function compileFirmwareFunctions(source, names, globals = {}, options = {}) {
  let javascript = names
    .map((name) => replaceSignature(extractFunction(source, name), name))
    .join("\n\n");

  javascript = javascript
    .replace(/\/\*[\s\S]*?\*\//g, "")
    .replace(/\/\/[^\n]*/g, "")
    .replace(/^\s*#.*$/gm, "")
    .replace(
      /static\s+const\s+char\s*\*\s*(\w+)\s*\[\d+\]\s*=\s*\{([\s\S]*?)\};/g,
      "const $1 = [$2];",
    )
    .replace(/\btm\s+(\w+)\s*\{\s*\}\s*;/g, "let $1 = {};")
    .replace(
      /\b(const\s+)?(?:struct\s+tm\s*\*|unsigned\s+long|long|u?int(?:8|16|32|64)?_t|int|float|double|bool|time_t|char)\s+(\w+)\s*=/g,
      (_match, constant, variable) => `${constant ? "const" : "let"} ${variable} =`,
    )
    .replace(/\bfor\s*\(\s*int\s+(\w+)\s*=/g, "for (let $1 =")
    .replace(/\((?:unsigned\s+long|u?int(?:8|16|32|64)?_t|int|float|double|bool|gpio_num_t)\)/g, "")
    .replace(/(\d+(?:\.\d+)?)(?:ULL|LLU|UL|LU|U|L|f)\b/gi, "$1")
    .replace(/\bisfinite\s*\(/g, "Number.isFinite(")
    .replace(/\bfabsf\s*\(/g, "Math.abs(")
    .replace(/\blroundf\s*\(/g, "Math.round(")
    .replace(/\bfloor\s*\(/g, "Math.floor(")
    .replace(/\bmin\s*\(/g, "Math.min(")
    .replace(/\bmax\s*\(/g, "Math.max(")
    .replace(/\bNAN\b/g, "NaN")
    .replace(/\bnullptr\b/g, "null");

  for (const [pattern, replacement] of options.replacements ?? []) {
    javascript = javascript.replace(pattern, replacement);
  }

  const globalNames = Object.keys(globals);
  const globalValues = Object.values(globals);
  try {
    const factory = new Function(
      ...globalNames,
      `"use strict";\n${javascript}\nreturn { ${names.join(", ")} };`,
    );
    return factory(...globalValues);
  } catch (error) {
    error.message = `${error.message}\n\nTranspiled firmware:\n${javascript}`;
    throw error;
  }
}
