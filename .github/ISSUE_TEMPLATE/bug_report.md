---
name: Bug report
about: Create a report to help us improve
title: ''
labels: ''
assignees: ''

body:
  - type: input
    attributes:
      label: Version
      description: What version of `tesseract` are you using?
      placeholder: Post a tag (e.g., `0.25.0`) or commit hash (e.g., `5ad42b6`)
    validations:
      required: True

  - type: dropdown
    attributes:
      label: OS Version
      description: What OS version are you running?
      options:
        - Ubuntu 20.04
        - Ubuntu 22.04
        - Ubuntu 24.04
        - MacOS 12
        - MacOS 14
        - Windows 10
        - Windows 11
        - Other (please specify in the bug description)
      validations:
        required: True

  - type: textarea
    attributes:
      label: Describe the bug
      placeholder: |
        A clear and concise description of the bug
    validations:
      required: True

  - type: textarea
    attributes:
      label: To Reproduce
      placeholder: | 
        Describe the steps to reproduce the behavior

  - type: textarea
    attributes:
      label: Expected behavior
      value: | 
        A clear and concise description of what you expected to happen.

  - type: textarea
    attributes:
      label: Relevant log output
      placeholder: |
        Paste log output here
      render: bash

---
