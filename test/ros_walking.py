#!/usr/bin/env python3
from python_op3.framework.modules.walking import RosRampWalk


def run(*args, **kwargs):
    op3 = RosRampWalk(*args, **kwargs)
    op3.run()
    # op3._update_walking_params()


if __name__ == '__main__':
    run("up_params.json", epi=0)
