from __future__ import annotations

from html import escape, unescape
import json
from pathlib import Path
import re
import sys
import unicodedata

ROOT = Path(__file__).resolve().parent
if (ROOT / '.deps').is_dir():
    sys.path.insert(0, str(ROOT / '.deps'))
import markdown

PAGES = {
    'index': ('首页', 'FOC 部署 Wiki'),
    '00-project': ('3.1 工程启动', '第 3 章 · 3.1'),
    '01-pwm-svpwm': ('3.2 PWM 与 SVPWM', '第 3 章 · 3.2'),
    '02-open-loop': ('3.3 电压开环部署', '第 3 章 · 3.3'),
    '03-sensors': ('3.4 电流与角度传感器', '第 3 章 · 3.4'),
    '04-sampling-zero': ('3.5 采样、中断与电零点', '第 3 章 · 3.5'),
    '05-transforms': ('3.6 坐标变换与开环验算', '第 3 章 · 3.6'),
    '06-current-pi': ('3.7 PI 电流闭环', '第 3 章 · 3.7'),
    '06a-software-architecture': ('3.8 软件分层', '第 3 章 · 3.8'),
    '07-speed': ('3.9 速度估计与速度闭环', '第 3 章 · 3.9'),
    '08-can-mit': ('3.10 CAN MIT', '第 3 章 · 3.10'),
    '09-lut': ('3.11 开环 LUT 标定', '第 3 章 · 3.11'),
    'reference': ('配置与验证参考', '附录'),
    'source': ('源码定位', '附录'),
}
READING = [page for page in PAGES if page != 'source']
DIST = ROOT / 'docs'


def slug(value, separator):
    value = ''.join(c for c in value.lower() if c in '-_' or not unicodedata.category(c).startswith(('P', 'S')))
    return re.sub(r'\s+', separator, value.strip())


def nav(current):
    out = ['<nav aria-label="课程导航">']
    for key, (name, _) in PAGES.items():
        if key in ('00-project', 'reference'):
            heading = '第 3 章 · FOC 实际部署' if key == '00-project' else '附录'
            out.append(f'<div class="nav-heading">{heading}</div>')
        marker = ' aria-current="page"' if key == current else ''
        out.append(f'<a href="{key}.html"{marker}>{name}</a>')
    return ''.join(out) + '</nav>'


def footer_nav(current):
    if current not in READING:
        return ''
    i = READING.index(current)
    before = READING[i - 1] if i else None
    after = READING[i + 1] if i < len(READING) - 1 else None
    if current == 'index':
        return ''
    prev = f'<a href="{before}.html"><small>上一页</small>{PAGES[before][0]}</a>' if before else ''
    nxt = f'<a href="{after}.html"><small>下一页</small>{PAGES[after][0]}</a>' if after else ''
    return f'<div class="pager">{prev}{nxt}</div>'


def plain(html):
    return re.sub(r'\s+', ' ', unescape(re.sub(r'<[^>]+>', ' ', html))).strip()


def render(name):
    text = (ROOT / 'content' / f'{name}.md').read_text(encoding='utf-8')
    top, text = text.split('\n', 1)
    assert top.startswith('# ')
    title = top[2:].strip()
    text = re.sub(r'(\]\()([a-z0-9-]+)\.md(?=[)#])', r'\1\2.html', text)
    md = markdown.Markdown(extensions=['extra', 'toc', 'sane_lists'],
                           extension_configs={'toc': {'slugify': slug, 'toc_depth': '2-3'}})
    body = md.convert(text)
    body = body.replace('<table>', '<div class="table-wrap"><table>').replace('</table>', '</table></div>')
    outline = md.toc
    initial = re.split(r'<h[23] id="[^"]+">', body, maxsplit=1)[0]
    index = [{'title': title, 'page': PAGES[name][0], 'url': f'{name}.html', 'text': plain(initial)}]
    for section in re.finditer(r'<h([23]) id="([^"]+)">(.*?)</h\1>(.*?)(?=<h[23] id=|\Z)', body, re.S):
        _, anchor, heading, html = section.groups()
        index.append({'title': plain(heading), 'page': PAGES[name][0], 'url': f'{name}.html#{anchor}', 'text': plain(html)})
    label = PAGES[name][1]
    header = f'<header class="article-header"><span class="eyebrow">{label}</span><h1>{escape(title)}</h1></header>'
    markup = f'''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<meta name="description" content="从 Simulink 仿真到 STM32 实机部署的 FOC 教程。">
<title>{escape(title)} · FOC 部署 Wiki</title>
<link rel="stylesheet" href="assets/wiki.css">
<script src="assets/search-index.js" defer></script>
<script src="assets/wiki.js" defer></script>
</head>
<body>
<a class="skip-link" href="#article">跳到正文</a>
<header class="masthead"><div class="masthead-inner">
<a class="brand" href="index.html">FOC 部署 Wiki</a><span class="edition">Simulink → STM32 · 第 3 章</span>
</div></header>
<div class="shell">
<aside class="sidebar">
<form class="search" role="search"><input type="search" aria-label="搜索课程" placeholder="搜索课程" autocomplete="off"><button type="submit">搜索</button></form>
<div class="search-results" aria-live="polite" hidden></div>
{nav(name)}
</aside>
<main class="article" id="article">
{header}
{body}
{footer_nav(name)}
</main>
<aside class="outline" aria-label="本页目录"><div class="nav-heading">本页目录</div>{outline}</aside>
</div>
<footer class="footer">FOC 部署 Wiki · 仿真、固件和实物</footer>
</body>
</html>
'''
    (DIST / f'{name}.html').write_text(markup, encoding='utf-8', newline='\n')
    return index


if __name__ == '__main__':
    (DIST / 'assets').mkdir(parents=True, exist_ok=True)
    index = [entry for page in PAGES for entry in render(page)]
    (DIST / 'assets' / 'search-index.js').write_text('window.WIKI_INDEX = ' + json.dumps(index, ensure_ascii=False) + ';\n', encoding='utf-8', newline='\n')
    (DIST / '.nojekyll').touch()
    print(f'Rendered {len(PAGES)} pages and {len(index)} search entries.')
