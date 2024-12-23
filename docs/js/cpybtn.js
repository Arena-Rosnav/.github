// Code taken from https://github.com/executablebooks/sphinx-copybutton

// Localization support
const messages = {
  en: {
    copy: "Copy",
    copy_to_clipboard: "Copy to clipboard",
    copy_success: "Copied!",
    copy_failure: "Failed to copy",
  },
  es: {
    copy: "Copiar",
    copy_to_clipboard: "Copiar al portapapeles",
    copy_success: "¡Copiado!",
    copy_failure: "Error al copiar",
  },
  de: {
    copy: "Kopieren",
    copy_to_clipboard: "In die Zwischenablage kopieren",
    copy_success: "Kopiert!",
    copy_failure: "Fehler beim Kopieren",
  },
  fr: {
    copy: "Copier",
    copy_to_clipboard: "Copié dans le presse-papier",
    copy_success: "Copié !",
    copy_failure: "Échec de la copie",
  },
  ru: {
    copy: "Скопировать",
    copy_to_clipboard: "Скопировать в буфер",
    copy_success: "Скопировано!",
    copy_failure: "Не удалось скопировать",
  },
  "zh-CN": {
    copy: "复制",
    copy_to_clipboard: "复制到剪贴板",
    copy_success: "复制成功!",
    copy_failure: "复制失败",
  },
  it: {
    copy: "Copiare",
    copy_to_clipboard: "Copiato negli appunti",
    copy_success: "Copiato!",
    copy_failure: "Errore durante la copia",
  },
};

let locale = "en";

/**
 * SVG files for our copy buttons
 */
let iconCheck = `<svg xmlns="http://www.w3.org/2000/svg" class="icon icon-tabler icon-tabler-check" width="44" height="44" viewBox="0 0 24 24" stroke-width="2" stroke="#22863a" fill="none" stroke-linecap="round" stroke-linejoin="round">
  <title>${messages[locale]["copy_success"]}</title>
  <path stroke="none" d="M0 0h24v24H0z" fill="none"/>
  <path d="M5 12l5 5l10 -10" />
</svg>`;

// If the user specified their own SVG use that, otherwise use the default
let iconCopy = `<svg xmlns="http://www.w3.org/2000/svg" class="icon icon-tabler icon-tabler-copy" width="44" height="44" viewBox="0 0 24 24" stroke-width="1.5" stroke="#000000" fill="none" stroke-linecap="round" stroke-linejoin="round">
  <title>${messages[locale]["copy_to_clipboard"]}</title>
  <path stroke="none" d="M0 0h24v24H0z" fill="none"/>
  <rect x="8" y="8" width="12" height="12" rx="2" />
  <path d="M16 8v-2a2 2 0 0 0 -2 -2h-8a2 2 0 0 0 -2 2v8a2 2 0 0 0 2 2h2" />
</svg>`;

/**
 * Set up copy/paste for code blocks
 */

const runWhenDOMLoaded = (cb) => {
  if (document.readyState != "loading") {
    cb();
  } else if (document.addEventListener) {
    document.addEventListener("DOMContentLoaded", cb);
  } else {
    document.attachEvent("onreadystatechange", function () {
      if (document.readyState == "complete") cb();
    });
  }
};

const codeCellId = (index) => `codecell${index}`;

// Clears selected text since ClipboardJS will select the text when copying
const clearSelection = () => {
  if (window.getSelection) {
    window.getSelection().removeAllRanges();
  } else if (document.selection) {
    document.selection.empty();
  }
};

// Changes tooltip text for two seconds, then changes it back
const temporarilyChangeTooltip = (el, oldText, newText) => {
  el.setAttribute("data-tooltip", newText);
  el.classList.add("success");
  setTimeout(() => el.setAttribute("data-tooltip", oldText), 2000);
  setTimeout(() => el.classList.remove("success"), 2000);
};

// Changes the copy button icon for two seconds, then changes it back
const temporarilyChangeIcon = (el) => {
  el.innerHTML = iconCheck;
  setTimeout(() => {
    el.innerHTML = iconCopy;
  }, 2000);
};

const addCopyButtonToCodeCells = () => {
  // If ClipboardJS hasn't loaded, wait a bit and try again. This
  // happens because we load ClipboardJS asynchronously.
  if (window.Clipboard === undefined) {
    setTimeout(addCopyButtonToCodeCells, 250);
    return;
  }

  // Add copybuttons to all of our code cells
  const codeCells = document.querySelectorAll("pre");
  codeCells.forEach((codeCell, index) => {
    var parent = codeCell.parentNode;
    var wrapper = document.createElement("div");
    wrapper.classList.add("codehilite");
    // set the wrapper as child (instead of the element)
    parent.replaceChild(wrapper, codeCell);
    // set element as child of wrapper
    wrapper.appendChild(codeCell);
    const id = codeCellId(index);
    codeCell.setAttribute("id", id);
    const clipboardButton = (id) =>
      `<button class="copybtn o-tooltip--left" data-tooltip="${messages[locale]["copy"]}" data-clipboard-target="#${id}">
      ${iconCopy}
    </button>`;
    codeCell.insertAdjacentHTML("afterend", clipboardButton(id));
  });

  var copyTargetText = (trigger) => {
    var target = document.querySelector(
      trigger.attributes["data-clipboard-target"].value
    );
    return target.innerText;
  };

  // Initialize with a callback so we can modify the text before copy
  const clipboard = new Clipboard(".copybtn", { text: copyTargetText });

  // Update UI with error/success messages
  clipboard.on("success", (event) => {
    clearSelection();
    temporarilyChangeTooltip(
      event.trigger,
      messages[locale]["copy"],
      messages[locale]["copy_success"]
    );
    temporarilyChangeIcon(event.trigger);
  });

  clipboard.on("error", (event) => {
    temporarilyChangeTooltip(
      event.trigger,
      messages[locale]["copy"],
      messages[locale]["copy_failure"]
    );
  });
};

runWhenDOMLoaded(addCopyButtonToCodeCells);
