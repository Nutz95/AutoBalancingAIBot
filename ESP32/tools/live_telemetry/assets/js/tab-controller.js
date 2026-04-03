export class TabController {
  constructor(documentRef = document) {
    this._document = documentRef;
    this._buttons = Array.from(this._document.querySelectorAll('[data-tab-button]'));
    this._panels = Array.from(this._document.querySelectorAll('[data-tab-panel]'));
    this._changeCallback = null;
  }

  start(changeCallback) {
    this._changeCallback = changeCallback;
    for (const button of this._buttons) {
      button.addEventListener('click', () => {
        this.activate(button.dataset.tabButton);
      });
    }
  }

  activate(tabName) {
    for (const button of this._buttons) {
      button.classList.toggle('is-active', button.dataset.tabButton === tabName);
    }
    for (const panel of this._panels) {
      panel.classList.toggle('is-active', panel.dataset.tabPanel === tabName);
    }
    if (this._changeCallback) {
      this._changeCallback(tabName);
    }
  }
}
