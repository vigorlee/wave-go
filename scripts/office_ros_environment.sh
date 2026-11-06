#!/usr/bin/env bash
# Private localhost router for this OfficeWorld experiment.
OFFICE_ROS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ZENOH_SESSION_CONFIG_URI="$OFFICE_ROS_ROOT/config/global_compare/zenoh_office_session.json5"
export ZENOH_ROUTER_CONFIG_URI="$OFFICE_ROS_ROOT/config/global_compare/zenoh_office_router.json5"
export ROAMERX_ZENOH_PORT=7759 GO2W_ZENOH_ROUTER_DOMAIN_ID=489
