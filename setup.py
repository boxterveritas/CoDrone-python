from setuptools import setup, find_packages

setup_requires = [
    ]

install_requires = [
	'pyserial',
	'numpy',
	'colorama'
    ]

dependency_links = [
    ]
desc = """\
this is Python package for control Codrone
"""

setup(
    name='CoDrone',
    version='1.1.1',
    description='Python package for CoDrone',
    url='https://github.com/RobolinkInc/CoDrone-python.git',
    author='Robolink',
    author_email='info@robolink.com',
    packages=["CoDrone"],
    keywords=['robolink','drone','codrone'],
    include_package_data=True,
    install_requires=install_requires,
    setup_requires=setup_requires,
    dependency_links=dependency_links,
    python_requires='>=3',
    )
