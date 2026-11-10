'use strict';

const form = document.querySelector('.search');
const input = form.querySelector('input');
const results = document.querySelector('.search-results');

function updateSearch() {
  const query = input.value.trim().toLowerCase();
  results.replaceChildren();
  results.hidden = !query;
  if (!query) return;
  const terms = query.split(/\s+/);
  const matches = (window.WIKI_INDEX || []).map(entry => {
    const title = entry.title.toLowerCase();
    const body = (entry.page + ' ' + entry.text).toLowerCase();
    if (!terms.every(term => title.includes(term) || body.includes(term))) return null;
    const score = terms.filter(term => title.includes(term)).length;
    return { ...entry, score };
  }).filter(Boolean).sort((a, b) => b.score - a.score).slice(0, 8);
  if (!matches.length) {
    results.textContent = '没有找到相关小节。';
    return;
  }
  for (const item of matches) {
    const link = document.createElement('a');
    link.href = item.url;
    link.textContent = item.title;
    const page = document.createElement('small');
    page.textContent = item.page;
    link.append(page);
    results.append(link);
  }
}
form.addEventListener('submit', event => {
  event.preventDefault();
  updateSearch();
});
input.addEventListener('input', updateSearch);
input.addEventListener('keydown', event => {
  if (event.key === 'Escape') {
    input.value = '';
    updateSearch();
  }
});
