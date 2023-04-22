from setuptools import setup

package_name = 'swap_pillar'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='entity014',
    maintainer_email='radlove.012@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detectAI = swap_pillar.detectPil:main',
            'detectManual = swap_pillar.detectColor:main'
        ],
    },
)
