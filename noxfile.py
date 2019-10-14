import nox
import os

VENV = 'conda' if os.name == 'nt' else None
NAME = 'win' if os.name == 'nt' else 'linux'
DISTS = ['sdist', 'bdist_wheel'] if os.name == 'nt' else ['sdist']

@nox.session(
    python=['3.6', '3.7'],
    reuse_venv=True,
    venv_backend=VENV,
    name='test_' + NAME)
def test(session):
    if VENV == 'conda':
        session.conda_install('--channel=conda-forge', 'shapely') # needed for windows
    session.install('-e', '.[dev]')
    session.run('pytest', 'tests/shape_test.py', '-n', '12')
    session.run('python', 'setup.py', *DISTS)
