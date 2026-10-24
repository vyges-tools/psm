// SPDX-License-Identifier: Apache-2.0
//! The database read: the tech table and one net, decoded from the opendb shim's flat streams.

use crate::input::{decode_net, decode_tech, NetInput, Tech};

pub fn tech(db: &vyges_opendb::Db) -> Result<Tech, String> {
    let raw = db.psm_tech_layers().map_err(|e| e.to_string())?;
    decode_tech(&raw, |n| db.layer_name_by_number(i64::from(n)))
}

pub fn net(db: &vyges_opendb::Db, name: &str) -> Result<NetInput, String> {
    let raw = db.psm_net(name).map_err(|e| e.to_string())?;
    let names = db.psm_net_names(name).map_err(|e| e.to_string())?;
    decode_net(name, &raw, &names)
}
