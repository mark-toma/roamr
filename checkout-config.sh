#!/usr/bin/env bash
cd $(dirname $0)
git checkout 2-add-initial-description-package roamr_description/config/roamr_geometry.yaml
git restore --staged roamr_description/config/roamr_geometry.yaml