from setuptools import setup, find_packages

setup(
    name='ospi',
    version='1.0',
    description='This library contains scripts for working with OpenSim files and pinocchio software.',
    long_description=read('README')
    keywords='biomechanics OpenSim pinocchio'
    url='https://github.com/GaloMALDONADO/ospi/',
    license='GNU General Public License v3.0',
    
    author='Galo MALDONADO',
    author_email='galo_xav@hotmail.com',

    packages=find_packages(exclude=['models','doc']),
    
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Science/Research',
        'Programming Language :: Python',
        'Programming Language :: Python :: 2.7',
    ],
    
)
