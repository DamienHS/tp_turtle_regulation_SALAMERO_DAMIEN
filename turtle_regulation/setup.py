from setuptools import setup

package_name = 'turtle_regulation'  # <-- underscore, pas de tiret

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Damien Salamero',
    maintainer_email='ton.email@example.com',
    description='Package de régulation en cap pour turtlesim',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'set_way_point    = turtle_regulation.set_way_point:main',
            'turtle_regulator = turtle_regulation.turtle_regulator:main',
        ],
    },
)

