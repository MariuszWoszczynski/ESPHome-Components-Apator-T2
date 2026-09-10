/*
 Copyright (C) 2017-2023 Fredrik Öhrström (gpl-3.0-or-later)

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include"meters.h"
#include"meters_common_implementation.h"
#include"units.h"
#include"wmbus.h"
#include"wmbus_utils.h"

#include<algorithm>
#include<cmath>
#include<limits>
#include<memory.h>
#include<numeric>
#include<stdexcept>
#include<time.h>

std::map<std::string, DriverInfo> *registered_drivers_ = NULL;
std::vector<DriverInfo*> *registered_drivers_list_ = NULL;
std::map<std::string, std::string> removed_driver_explanation_;

void verifyDriverLookupCreated()
{
    if (registered_drivers_ == NULL)
    {
        registered_drivers_ = new std::map<std::string,DriverInfo>;
    }
    if (registered_drivers_list_ == NULL)
    {
        registered_drivers_list_ = new std::vector<DriverInfo*>;
    }
}

DriverInfo *lookupDriver(std::string name)
{
    verifyDriverLookupCreated();

    if (registered_drivers_->count(name) == 1)
    {
        return &(*registered_drivers_)[name];
    }

    for (DriverInfo *di : *registered_drivers_list_)
    {
        for (DriverName &dn : di->nameAliases())
        {
            if (dn.str() == name)
            {
                return di;
            }
        }
    }

    return NULL;
}

std::vector<DriverInfo*> &allDrivers()
{
    verifyDriverLookupCreated();
    return *registered_drivers_list_;
}

void removeDriver(const std::string &name, std::string explanation)
{
    verifyDriverLookupCreated();
    for (auto i = registered_drivers_list_->begin(); i != registered_drivers_list_->end(); i++)
    {
        if ((*i)->name().str() == name)
        {
            registered_drivers_list_->erase(i);
            break;
        }
    }

    registered_drivers_->erase(name);
    removed_driver_explanation_[name] = explanation;
    assert(registered_drivers_->count(name) == 0);
}

std::string removedDriverExplanation(const std::string& name)
{
    if (removed_driver_explanation_.count(name) > 0)
    {
        return removed_driver_explanation_[name];
    }
    return "";
}

void addRegisteredDriver(DriverInfo di)
{
    verifyDriverLookupCreated();
    if (registered_drivers_->count(di.name().str()) != 0)
    {
        error("Two drivers trying to register the name \"%s\"\n", di.name().str().c_str());
        exit(1);
    }

    (*registered_drivers_)[di.name().str()] = di;
    (*registered_drivers_list_).push_back(lookupDriver(di.name().str()));
}

bool DriverInfo::detect(uint16_t mfct, uchar version, uchar type)
{
    for (auto &dd : mvts_)
    {
        if (dd.mfct == 0 && dd.type == 0 && dd.version == 0) continue;
        if ((dd.mfct & 0x7fff) == (mfct & 0x7fff) && dd.version == version && dd.type == type ) return true;
    }
    return false;
}

void DriverInfo::setAliases(string a)
{
    if (a == "") return;

    auto as = splitString(a, ',');
    for (string& s : as)
    {
        addNameAlias(s);
    }
}

bool DriverInfo::isValidMedia(uchar type)
{
    for (auto &dd : mvts_)
    {
        if (dd.type == type) return true;
    }
    return false;
}

DriverInfo::~DriverInfo()
{
}

bool DriverInfo::isCloseEnoughMedia(uchar type)
{
    for (auto &dd : mvts_)
    {
        if (isCloseEnough(dd.type, type)) return true;
    }
    return false;
}

bool staticRegisterDriver(function<void(DriverInfo&)> setup)
{
    DriverInfo di;
    setup(di);

    // Do not rely on assert() side effects here. ESP-IDF/ESPHome may compile
    // with assertions disabled, and driver registration runs from static
    // initializers before normal component setup.
    verifyDriverLookupCreated();
    assert(lookupDriver(di.name().str()) == NULL);

    for (auto &d : di.mvts())
    {
        for (DriverInfo *p : allDrivers())
        {
            bool foo = p->detect(d.mfct, d.version, d.type);
            if (foo)
            {
                error("Internal error: driver %s tried to register the same auto detect combo as driver %s alread has taken!\n",
                      di.name().str().c_str(), p->name().str().c_str());
            }
        }
    }

    addRegisteredDriver(di);
    return true;
}

bool lookupDriverInfo(const std::string& driver_name, DriverInfo *out_di)
{
    DriverInfo *di = lookupDriver(driver_name);
    if (di)
    {
        if (out_di) *out_di = *di;
        return true;
    }

    return false;
}

