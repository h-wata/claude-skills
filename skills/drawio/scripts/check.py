# ruff: noqa: CPY001  個人リポジトリなので会社の著作権表記は付けない
""".drawio の XML を調べる: 読めるか・id の重複・線の接続先が存在するか."""

import sys
import xml.etree.ElementTree as ET


def check(path: str) -> list[str]:
    errors = []
    for diagram in ET.parse(path).getroot().iter('diagram'):
        cells = list(diagram.iter('mxCell'))
        ids = [c.get('id') for c in cells]
        errors += [f'{diagram.get("name")}: id {i} が重複' for i in sorted({i for i in ids if ids.count(i) > 1})]
        for c in cells:
            if c.get('edge') != '1':
                continue
            for end in ('source', 'target'):
                if c.get(end) and c.get(end) not in ids:
                    errors.append(f'{diagram.get("name")}: 線 {c.get("id")} の {end}={c.get(end)} が存在しない')
    return errors


if __name__ == '__main__':
    try:
        errors = check(sys.argv[1])
    except ET.ParseError as e:
        errors = [f'XML として読めない: {e}']
    print('\n'.join(errors) or 'OK')
    sys.exit(1 if errors else 0)
